#pragma once

#include <cmath>
#include <functional>
#include <iostream>
#include <mutex>
#include <thread>

#include "oidn.hpp"

#include "Core.h"
#include "GamesEngineeringBase.h"
#include "Geometry.h"
#include "Imaging.h"
#include "Lights.h"
#include "Materials.h"
#include "Sampling.h"
#include "Scene.h"

struct ScreenTile {
	unsigned int tile_x{ 0 }, tile_y{ 0 };
	unsigned int tile_size{ 32 };
	std::atomic<bool> is_tile_rendered = false;

	// Get start index of x and y
	unsigned int tile_x_start() const { return std::max(static_cast<unsigned int>(0), tile_x); }
	unsigned int tile_y_start() const { return std::max(static_cast<unsigned int>(0), tile_y); }

	// Get end index of x and y
	unsigned int tile_x_end(Film* film) const { return std::min(tile_x + tile_size - 1, film->width - 1); }
	unsigned int tile_y_end(Film* film) const { return std::min(tile_y + tile_size - 1, film->height - 1); }
};

class RayTracer {
public:
	Scene* scene;
	GamesEngineeringBase::Window* canvas;
	MTRandom* samplers;
	std::thread** threads;
	int numProcs;

	// OIDN
	oidn::DeviceRef device;
	oidn::FilterRef filter;
	oidn::BufferRef colourBuf;
	oidn::BufferRef albedoBuf;
	oidn::BufferRef normalBuf;
	oidn::BufferRef outputBuf;

	Film* film;
	Film* normalFilm;
	Film* albedoFilm;
	
	void init(Scene* _scene, GamesEngineeringBase::Window* _canvas) {
		scene = _scene;
		canvas = _canvas;
		film = new Film();
		film->init((unsigned int)scene->camera.width, (unsigned int)scene->camera.height, new BoxFilter());
		// film->init((unsigned int)scene->camera.width, (unsigned int)scene->camera.height, new GaussianFilter(1.5f, 0.5f));
		// film->init((unsigned int)scene->camera.width, (unsigned int)scene->camera.height, new MitchellNetravali());
		SYSTEM_INFO sysInfo;
		GetSystemInfo(&sysInfo);
		numProcs = sysInfo.dwNumberOfProcessors;
		threads = new std::thread * [numProcs];
		samplers = new MTRandom[numProcs];

		// OIDN
		device = oidn::newDevice();
		device.commit();

		// Check for errors
		const char* errorMessage;
		if (device.getError(errorMessage) != oidn::Error::None) std::cout << "Error: " << errorMessage << std::endl;

		colourBuf = device.newBuffer(film->width * film->height * 3 * sizeof(float));
		albedoBuf = device.newBuffer(film->width * film->height * 3 * sizeof(float));
		normalBuf = device.newBuffer(film->width * film->height * 3 * sizeof(float));
		outputBuf = device.newBuffer(film->width * film->height * 3 * sizeof(float));

		filter = device.newFilter("RT");
		filter.setImage("color", colourBuf, oidn::Format::Float3, film->width, film->height);
		filter.setImage("albedo", albedoBuf, oidn::Format::Float3, film->width, film->height);
		filter.setImage("normal", normalBuf, oidn::Format::Float3, film->width, film->height);
		filter.setImage("output", outputBuf, oidn::Format::Float3, film->width, film->height);
		filter.set("hdr", true);
		filter.commit();

		normalFilm = new Film();
		normalFilm->init((unsigned int)scene->camera.width, (unsigned int)scene->camera.height, new BoxFilter());
		
		albedoFilm = new Film();
		albedoFilm->init((unsigned int)scene->camera.width, (unsigned int)scene->camera.height, new BoxFilter());
		
		clear();
	}

	void clear() {
		film->clear();
	}

	Colour computeDirect(ShadingData shadingData, Sampler* sampler) {
		// Is surface is specular we cannot computing direct lighting
		if (shadingData.bsdf->isPureSpecular() == true) {
			return Colour(0.0f, 0.0f, 0.0f);
		}
		// Compute direct lighting here
		// Sample a light
		float pdf, pmf;
		Colour emission;
		Light* light = scene->sampleLight(sampler, pmf);

		// Area Light
		if (light->isArea()) {
			// Sample point on light and store returned emission
			Vec3 samplePointOnLight = light->sample(shadingData, sampler, emission, pdf);

			// Calculate Geometry Term
			Vec3 surfaceToLight = samplePointOnLight - shadingData.x;
			Vec3 wi = surfaceToLight.normalize();
			float gTerm = (std::max(Dot(wi, shadingData.sNormal), 0.f) * std::max(-Dot(wi, light->normal(shadingData, wi)), 0.f)) / surfaceToLight.lengthSq();

			// Calculate Visibility: V[x(i) <-> x(i+1)] (Binary function, from Ray Tracing)
			if (scene->visible(shadingData.x, samplePointOnLight)) {
				// Calculate BSDF
				Colour BSDF = shadingData.bsdf->evaluate(shadingData, wi);
				
				// Calculate Weight for MIS (Power Heuristic)
				float bsdfPDF = shadingData.bsdf->PDF(shadingData, wi);
				float cosTheta = Dot(wi, shadingData.sNormal);
				float pLight = pdf * pmf;
				float pBsdf = bsdfPDF * gTerm / cosTheta;
				float w = powerHeuristics(pLight, pBsdf, 2);

				// Return the integral
				return emission * BSDF * gTerm * w / pLight;
			}
			return Colour(0.f, 0.f, 0.f);
		}
		// Environment Map
		else {
			// Sample from light, returns direction instead of point
			Vec3 wi = light->sample(shadingData, sampler, emission, pdf);
			
			// Evaluate Geometry Term for Environment Maps
			float gTerm = std::max(Dot(wi, shadingData.sNormal), 0.f);
			 
			// Evaluate Visibility to out-of-scene bounds
			if (scene->visible(shadingData.x + (wi * EPSILON), shadingData.x + (wi * use<SceneBounds>().sceneRadius))) {
				// Evaluate BSDF, multiply term, and return
				Colour BSDF = shadingData.bsdf->evaluate(shadingData, wi);
				float bsdfPDF = shadingData.bsdf->PDF(shadingData, wi);
				float cosTheta = Dot(wi, shadingData.sNormal);
				float pLight = pdf * pmf;
				float pBsdf = bsdfPDF * gTerm / cosTheta;
				float w = powerHeuristics(pLight, pBsdf, 2);
				return emission * BSDF * gTerm * w / pLight;
			}
			return Colour(0.f, 0.f, 0.f);
		}
	}

	Colour direct(Ray& r, Sampler* sampler) {
		// Compute direct lighting for an image sampler here
		IntersectionData intersection = scene->traverse(r);
		ShadingData shadingData = scene->calculateShadingData(intersection, r);
		if (shadingData.t < FLT_MAX) {
			if (shadingData.bsdf->isLight()) {
				return shadingData.bsdf->emit(shadingData, shadingData.wo);
			}
			return computeDirect(shadingData, sampler);
		}
		return scene->background->evaluate(r.dir);
	}

	Colour pathTrace(Ray& r, Sampler* sampler) {
		Colour pathThroughput(1.f, 1.f, 1.f);
		return pathTraceRecursive(r, pathThroughput, 0, sampler);
	}

	Colour pathTraceRecursive(Ray& r, Colour& pathThroughput, int depth, Sampler* sampler) {
		// Add pathtracer code here
		// Trace Ray
		IntersectionData intersection = scene->traverse(r);
		ShadingData shadingData = scene->calculateShadingData(intersection, r);
		if (shadingData.t < FLT_MAX) {
			if (shadingData.bsdf->isLight()) {
				return (depth == 0) ? shadingData.bsdf->emit(shadingData, shadingData.wo) : Colour(0.f, 0.f, 0.f);
			}
			// Calculate Direct Lighting
			Colour direct = pathThroughput * computeDirect(shadingData, sampler);
			// if (depth >= 10) return direct;

			// Sample Indirect Direction
			// Vec3 incomingRadiance = SamplingDistributions::cosineSampleHemisphere(sampler->next(), sampler->next());
			// float pdf = SamplingDistributions::cosineHemispherePDF(incomingRadiance);
			// incomingRadiance = shadingData.frame.toWorld(incomingRadiance);
			// Colour indirect = shadingData.bsdf->evaluate(shadingData, wi);
			
			float pdf;
			Colour indirect;
			Vec3 wi = shadingData.bsdf->sample(shadingData, sampler, indirect, pdf);
			Ray indirectRay(shadingData.x + (wi * EPSILON), wi);

			// Update path throughput (multiply with BSDF and cosine, divide by pdf)
			float cosine = Dot(wi, shadingData.sNormal);
			//float indirectBsdfPDF = shadingData.bsdf->PDF(shadingData, wi);
			//float indirectBsdfPDFArea = indirectBsdfPDF * std::max(Dot(wi, shadingData.x), 0.f);
			//float wIndirect = powerHeuristics(indirectBsdfPDFArea, pdf);
			pathThroughput = pathThroughput * indirect * cosine /** wIndirect *// pdf;

			// Apply Russian Roulette
			if (depth >= 3) {
				float rrp = std::min(pathThroughput.Lum(), 0.995f);
				if (sampler->next() < rrp) {
					pathThroughput = pathThroughput / rrp;
					return direct + pathTraceRecursive(indirectRay, pathThroughput, depth + 1, sampler);
				}
				else return direct;
			}
			return direct + pathTraceRecursive(indirectRay, pathThroughput, depth + 1, sampler);
		}
		return scene->background->evaluate(r.dir) * pathThroughput;
	}

	Colour albedo(Ray& r) {
		IntersectionData intersection = scene->traverse(r);
		ShadingData shadingData = scene->calculateShadingData(intersection, r);
		if (shadingData.t < FLT_MAX) {
			if (shadingData.bsdf->isLight()) {
				return shadingData.bsdf->emit(shadingData, shadingData.wo);
			}
			return shadingData.bsdf->evaluate(shadingData, Vec3(0, 1, 0));
		}
		return scene->background->evaluate(r.dir);
	}

	Colour viewNormals(Ray& r) {
		IntersectionData intersection = scene->traverse(r);
		if (intersection.t < FLT_MAX) {
			ShadingData shadingData = scene->calculateShadingData(intersection, r);
			return Colour(fabsf(shadingData.sNormal.x), fabsf(shadingData.sNormal.y), fabsf(shadingData.sNormal.z));
		}
		return Colour(0.0f, 0.0f, 0.0f);
	}

	void denoise() {
		unsigned int pixels = film->width * film->height;
		float* normalPtr = (float*)normalBuf.getData();
		for (unsigned int i = 0; i < pixels; i++) {
			normalPtr[(i * 3)] = std::min(std::max(0.f, normalFilm->film[i].r), 255.f);
			normalPtr[(i * 3) + 1] = std::min(std::max(0.f, normalFilm->film[i].g), 255.f);
			normalPtr[(i * 3) + 2] = std::min(std::max(0.f, normalFilm->film[i].b), 255.f);
		}

		float* albedoPtr = (float*)albedoBuf.getData();
		for (unsigned int i = 0; i < pixels; i++) {
			albedoPtr[(i * 3)] = std::min(std::max(0.f, albedoFilm->film[i].r), 255.f);
			albedoPtr[(i * 3) + 1] = std::min(std::max(0.f, albedoFilm->film[i].g), 255.f);
			albedoPtr[(i * 3) + 2] = std::min(std::max(0.f, albedoFilm->film[i].b), 255.f);
		}

		float* colourPtr = (float*)colourBuf.getData();
		for (unsigned int i = 0; i < pixels; i++) {
			colourPtr[(i * 3)] = std::min(std::max(0.f, film->film[i].r), 255.f);
			colourPtr[(i * 3) + 1] = std::min(std::max(0.f, film->film[i].g), 255.f);
			colourPtr[(i * 3) + 2] = std::min(std::max(0.f, film->film[i].b), 255.f);
		}

		// Execute Denoising
		filter.execute();
		float* outputPtr = (float*)outputBuf.getData();
		for (unsigned int i = 0; i < pixels; i++) {
			film->film[i].r = std::min(std::max(outputPtr[(i * 3)], 0.f), 255.f);
			film->film[i].g = std::min(std::max(outputPtr[(i * 3) + 1], 0.f), 255.f);
			film->film[i].b = std::min(std::max(outputPtr[(i * 3) + 2], 0.f), 255.f);
		}
	}

	void render() {
		// General Render Function to Select Desired Rendering Method
		int renderMode = 1;
		if (renderMode == 0) renderSequential();
		if (renderMode == 1) renderMultithread();
		if (renderMode == 2) renderMultithreadDenoise();
		if (renderMode == 3) renderLightTrace();
		if (renderMode == 4) renderInstantRadiosity();
		// if (renderMode == 5) renderPSSMLT();
	}

	void renderSequential() {
		// Sequential Rendering
		film->incrementSPP();
		for (unsigned int y = 0; y < film->height; y++) {
			for (unsigned int x = 0; x < film->width; x++) {
				float px = x + samplers->next();  // + 0.5f;
				float py = y + samplers->next();  // + 0.5f;
				Ray ray = scene->camera.generateRay(px, py);
				//Colour col = viewNormals(ray);
				//Colour col = albedo(ray);
				//Colour col = direct(ray, samplers);
				Colour col = pathTrace(ray, samplers);
				film->splat(px, py, col);
				unsigned char r, g, b;
				film->tonemap(x, y, r, g, b, 2.f);
				canvas->draw(x, y, r, g, b);
			}
		}
	}

	void renderMultithread() {
		// Multithreaded Tiled Rendering
		film->incrementSPP();
		std::mutex mtx;
		std::vector<std::thread> thread_pool;
		std::atomic<int> atomic_id_counter = 0;
		thread_pool.reserve(numProcs);

		unsigned int tile_size = 32;
		unsigned int tile_count = (unsigned int)(std::ceil((film->width + tile_size - 1) / tile_size) * std::ceil((film->height + tile_size - 1) / tile_size));

		for (unsigned int i = 0; i < numProcs; ++i) {
			thread_pool.emplace_back([&]() {
				// Work function
				ScreenTile screen_tile;
				unsigned int tile_id = 0;

				while ((tile_id = atomic_id_counter.fetch_add(1)) < tile_count) {
					// Initialize ScreenTile structure's attributes
					{
						std::lock_guard<std::mutex> lock(mtx);
						screen_tile.tile_x = (tile_id % (unsigned int)std::ceil((film->width + tile_size - 1) / tile_size)) * tile_size;
						screen_tile.tile_y = (tile_id / (unsigned int)std::ceil((film->width + tile_size - 1) / tile_size)) * tile_size;
						screen_tile.tile_size = tile_size;
						screen_tile.is_tile_rendered = false;
					}

					if (!screen_tile.is_tile_rendered.load(std::memory_order_relaxed)) {
						for (unsigned int y = screen_tile.tile_y_start(); y <= screen_tile.tile_y_end(film); ++y) {
							for (unsigned int x = screen_tile.tile_x_start(); x <= screen_tile.tile_x_end(film); ++x) {
								float px = x + samplers->next();  // + 0.5f;
								float py = y + samplers->next();  // + 0.5f;
								Ray ray = scene->camera.generateRay(px, py);
								//Colour col = viewNormals(ray);
								//Colour col = albedo(ray);
								//Colour col = direct(ray, samplers);
								Colour col = pathTrace(ray, samplers);
								film->splat(px, py, col);
								unsigned char r, g, b;
								film->tonemap(x, y, r, g, b, 2.f);
								canvas->draw(x, y, r, g, b);
							}
						}

						// Tile rendered, update is_tile_rendered flag
						{
							std::lock_guard<std::mutex> lock(mtx);
							screen_tile.is_tile_rendered.store(true);
						}
					}
				}
			});
		}

		// Join the threads to ensure work completed
		for (auto& thread : thread_pool)
			thread.join();
	}

	void renderMultithreadDenoise() {
		// Multithreaded Tiled Rendering and Intel Open Image Denoising
		film->incrementSPP();
		std::mutex mtx;
		std::vector<std::thread> thread_pool;
		std::atomic<int> atomic_id_counter = 0;
		thread_pool.reserve(numProcs);

		unsigned int tile_size = 32;
		unsigned int tile_count = (unsigned int)(std::ceil((film->width + tile_size - 1) / tile_size) * std::ceil((film->height + tile_size - 1) / tile_size));

		for (unsigned int i = 0; i < numProcs; ++i) {
			thread_pool.emplace_back([&]() {
				// Work function
				ScreenTile screen_tile;
				unsigned int tile_id = 0;

				while ((tile_id = atomic_id_counter.fetch_add(1)) < tile_count) {
					// Initialize ScreenTile structure's attributes
					{
						std::lock_guard<std::mutex> lock(mtx);
						screen_tile.tile_x = (tile_id % (unsigned int)std::ceil((film->width + tile_size - 1) / tile_size)) * tile_size;
						screen_tile.tile_y = (tile_id / (unsigned int)std::ceil((film->width + tile_size - 1) / tile_size)) * tile_size;
						screen_tile.tile_size = tile_size;
						screen_tile.is_tile_rendered = false;
					}

					if (!screen_tile.is_tile_rendered.load(std::memory_order_relaxed)) {
						for (unsigned int y = screen_tile.tile_y_start(); y <= screen_tile.tile_y_end(film); ++y) {
							for (unsigned int x = screen_tile.tile_x_start(); x <= screen_tile.tile_x_end(film); ++x) {
								float px = x + samplers->next();  // + 0.5f;
								float py = y + samplers->next();  // + 0.5f;
								Ray ray = scene->camera.generateRay(px, py);
								// Preparing the films for Denoising
								Colour normalCol = viewNormals(ray);
								normalFilm->splat(px, py, normalCol);
								Colour albedoCol = albedo(ray);
								albedoFilm->splat(px, py, albedoCol);
								Colour pathCol = pathTrace(ray, samplers);
								film->splat(px, py, pathCol);
							}
						}

						// Tile rendered, update is_tile_rendered flag
						{
							std::lock_guard<std::mutex> lock(mtx);
							screen_tile.is_tile_rendered.store(true);
						}
					}
				}
			});
		}

		// Join the threads to ensure work completed
		for (auto& thread : thread_pool)
			thread.join();

		// Denoise and then Tone Map
		denoise();
		for (unsigned int y = 0; y < film->height; y++) {
			for (unsigned int x = 0; x < film->width; x++) {
				unsigned char r, g, b;
				film->tonemap(x, y, r, g, b, 2.f);
				canvas->draw(x, y, r, g, b);
			}
		}
	}

	void renderLightTrace() {
		// TO:DO
		film->incrementSPP();
	}

	void renderInstantRadiosity() {
		// TO:DO
		film->incrementSPP();
	}

	int getSPP() {
		return film->SPP;
	}

	void saveHDR(std::string filename) { 
		film->save(filename);
	}

	void savePNG(std::string filename) {
		stbi_write_png(filename.c_str(), canvas->getWidth(), canvas->getHeight(), 3, canvas->getBackBuffer(), canvas->getWidth() * 3);
	}
};