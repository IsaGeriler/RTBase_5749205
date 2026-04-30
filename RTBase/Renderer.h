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

class VPL {
public:
	ShadingData shadingData;
	Colour Le;

	// – For area lights, initialize with
	// ShadingData(p, light->normal(p));
	// – Where
	// p = light->samplePositionFromLight(sampler, pdfPosition);

};

// Second pass (can be parallelized)
// For each pixel
// • Trace ray
// • Iterate over all stored VPLs
// • Compute Contribution


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

		normalFilm = new Film();
		normalFilm->init((unsigned int)scene->camera.width, (unsigned int)scene->camera.height, new BoxFilter());

		albedoFilm = new Film();
		albedoFilm->init((unsigned int)scene->camera.width, (unsigned int)scene->camera.height, new BoxFilter());

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
		albedoBuf = device.newBuffer(albedoFilm->width * albedoFilm->height * 3 * sizeof(float));
		normalBuf = device.newBuffer(normalFilm->width * normalFilm->height * 3 * sizeof(float));
		outputBuf = device.newBuffer(film->width * film->height * 3 * sizeof(float));

		filter = device.newFilter("RT");
		filter.setImage("color", colourBuf, oidn::Format::Float3, film->width, film->height);
		filter.setImage("albedo", albedoBuf, oidn::Format::Float3, albedoFilm->width, albedoFilm->height);
		filter.setImage("normal", normalBuf, oidn::Format::Float3, normalFilm->width, normalFilm->height);
		filter.setImage("output", outputBuf, oidn::Format::Float3, film->width, film->height);
		filter.set("hdr", true);
		filter.commit();
		
		clear();
	}

	void clear() {
		film->clear();
		normalFilm->clear();
		albedoFilm->clear();
	}

	Colour computeDirect(ShadingData shadingData, Sampler* sampler) {
		// If surface is specular we cannot computing direct lighting
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
			if (pmf <= 0.f) return Colour(0.f, 0.f, 0.f);

			// Calculate Geometry Term
			Vec3 surfaceToLight = samplePointOnLight - shadingData.x;
			Vec3 wi = surfaceToLight.normalize();
			if (surfaceToLight.lengthSq() < EPSILON) return Colour(0.f, 0.f, 0.f);
			float gTerm = (std::max(Dot(wi, shadingData.sNormal), 0.f) * std::max(-Dot(wi, light->normal(shadingData, wi)), 0.f)) / surfaceToLight.lengthSq();

			// Calculate Visibility: V[x(i) <-> x(i+1)] (Binary function, from Ray Tracing)
			if (scene->visible(shadingData.x, samplePointOnLight)) {
				// Calculate BSDF
				Colour BSDF = shadingData.bsdf->evaluate(shadingData, wi);
				
				// Calculate Weight for MIS (Power Heuristic)
				float bsdfPDF = shadingData.bsdf->PDF(shadingData, wi);
				float cosine = std::max(Dot(wi, shadingData.sNormal), EPSILON);
				float pLight = pdf * pmf;
				float pBsdf = bsdfPDF * gTerm / cosine;
				float w = powerHeuristics(pLight, pBsdf, 2);

				// Return the integral
				if (pdf <= 0) return Colour(0.f, 0.f, 0.f);
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
			
			// Calculate Visibility to out-of-scene bounds (Binary function, from Ray Tracing)
			Vec3 sceneBounds = scene->bounds.max - scene->bounds.min;
			if (scene->visible(shadingData.x + (wi * EPSILON), shadingData.x + (wi * sceneBounds.length()))) {
				// Calculate BSDF
				Colour BSDF = shadingData.bsdf->evaluate(shadingData, wi);
				
				// Calculate Weight for MIS (Power Heuristic)
				float bsdfPDF = shadingData.bsdf->PDF(shadingData, wi);
				float cosine = std::max(Dot(wi, shadingData.sNormal), EPSILON);
				float pLight = pdf * pmf;
				float pBsdf = bsdfPDF * gTerm / cosine;
				float w = powerHeuristics(pLight, pBsdf, 2);
				
				// Return the integral
				if (pdf <= 0) return Colour(0.f, 0.f, 0.f);
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

	// Recheck... probably something is wrong here
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
			// if (depth > 10) return direct;

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
			pathThroughput = pathThroughput * indirect * cosine / pdf;

			// Apply Russian Roulette
			if (depth >= 3) {
				float rrp = std::min(pathThroughput.Lum(), 0.99f);
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

	// Light Tracing
	// Handle connections to camera
	void connectToCamera(Vec3 p, Vec3 n, Colour col) {
		// Check if p is on camera
		float x, y;
		if (scene->camera.projectOntoCamera(p, x, y)) {
			// Compute geometry term between p and camera
			Vec3 cNormal = scene->camera.viewDirection;  // Camera normal
			Vec3 cPos = scene->camera.origin;			 // Camera position
			Vec3 cDirection = cPos - p;					 // Direction to camera

			if (scene->visible(p, cPos)) {
				Vec3 wi = cDirection.normalize();
				if (cDirection.lengthSq() < EPSILON) return;

				// Need to compute We(x,w) = 1.f / (Afilm * cos4Theta)
				// Theta is angle between scene->camera.viewDirection and direction to camera
				float cosTheta = std::max(Dot(cNormal, -cDirection), EPSILON);
				float Afilm = scene->camera.Afilm;
				float W = 1.f / (Afilm * powf(cosTheta, 4));

				// Splat col onto film at coordinates from projectOntoCamera
				film->splat(x, y, col * W);
			}
		}
	}

	// Handles starting a light path
	void lightTrace(Sampler* sampler) {
		// Sample a light source
		float pmf;
		Light* light = scene->sampleLight(sampler, pmf);
		
		// Area Light
		if (light->isArea()) {
			// Sample point and direction from on light source
			float pdfDirection, pdfPosition;
			Vec3 p = light->samplePositionFromLight(sampler, pdfPosition);
			Vec3 wi = light->sampleDirectionFromLight(sampler, pdfDirection);
			
			// Connect position to camera
			Colour col = light->evaluate(-wi) / pdfPosition;
			Colour pathThroughput(1.f, 1.f, 1.f);
			connectToCamera(p, wi, col);
			
			// Create a ray starting at p in direction wi
			Ray r(p + (wi * EPSILON), wi);

			// Then call
			lightTracePath(r, pathThroughput, col, sampler, 0);
		}
		// Environment Map
		else {
			// Need to flip direction to trace into scene
			float pdfBounds, pdfDirection;
			Vec3 wi = -(light->sampleDirectionFromLight(sampler, pdfDirection));
			
			// Sample position on the scene
			Vec3 sampledPos = light->samplePositionFromLight(sampler, pdfBounds);
			
			// Overall PDF is product of Direction PDF and Position on bounds PDF
			float pdf = pdfBounds + pdfDirection;

			// Connect position to camera
			Colour col = light->evaluate(-wi) / pdf;
			Colour pathThroughput(1.f, 1.f, 1.f);
			connectToCamera(sampledPos, wi, col);

			// Create a ray starting at p in direction wi
			Ray r(sampledPos + (wi * EPSILON), wi);

			// Then call
			lightTracePath(r, pathThroughput, col, sampler, 0);
		}
	}

	// Handles tracing the rest of the light path
	void lightTracePath(Ray& r, Colour pathThroughput, Colour Le, Sampler* sampler, int depth) {
		IntersectionData intersection = scene->traverse(r);
		ShadingData shadingData = scene->calculateShadingData(intersection, r);
		if (shadingData.t < FLT_MAX) {
			if (shadingData.bsdf->isLight()) return;
			// Use direction to camera (need to normalize)
			Vec3 dirToCamera = scene->camera.origin - shadingData.x;
			dirToCamera.normalize();

			// col has value of pathThroughput * shadingData.bsdf->evaluate(shadingData , wi) * Le
			Colour col = pathThroughput * shadingData.bsdf->evaluate(shadingData, dirToCamera) * Le;
			
			// Connect each intersection to camera
			connectToCamera(shadingData.x, dirToCamera, col);
			
			// Apply Russian Roulette
			if (depth >= 3) {
				float rrp = std::min(pathThroughput.Lum(), 0.95f);
				if (sampler->next() < rrp) pathThroughput = pathThroughput / rrp;
				else return;
			}

			// Similar to path tracing but no direct lighting
			float pdf;
			Colour indirect;
			Vec3 wi = shadingData.bsdf->sample(shadingData, sampler, indirect, pdf);
			Ray indirectRay(shadingData.x + (wi * EPSILON), wi);

			// Update path throughput (multiply with IndirectBSDF and cosine, divide by pdf)
			float cosine = std::max(Dot(wi, shadingData.sNormal), EPSILON);
			pathThroughput = pathThroughput * indirect * cosine / pdf;
			lightTracePath(indirectRay, pathThroughput, Le, sampler, depth + 1);
		}
	}

	// Instant Radiosity
	// TO:DO

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
			normalPtr[(i * 3)] = normalFilm->film[i].r;
			normalPtr[(i * 3) + 1] = normalFilm->film[i].g;
			normalPtr[(i * 3) + 2] = normalFilm->film[i].b;
		}

		float* albedoPtr = (float*)albedoBuf.getData();
		for (unsigned int i = 0; i < pixels; i++) {
			albedoPtr[(i * 3)] = albedoFilm->film[i].r;
			albedoPtr[(i * 3) + 1] = albedoFilm->film[i].g;
			albedoPtr[(i * 3) + 2] = albedoFilm->film[i].b;
		}

		float* colourPtr = (float*)colourBuf.getData();
		for (unsigned int i = 0; i < pixels; i++) {
			colourPtr[(i * 3)] = film->film[i].r;
			colourPtr[(i * 3) + 1] = film->film[i].g;
			colourPtr[(i * 3) + 2] = film->film[i].b;
		}

		// Execute Denoising
		filter.execute();
		float* outputPtr = (float*)outputBuf.getData();
		for (unsigned int i = 0; i < pixels; i++) {
			film->film[i].r = std::max(std::min(outputPtr[(i * 3)], 255.f), 0.f);
			film->film[i].g = std::max(std::min(outputPtr[(i * 3) + 1], 255.f), 0.f);
			film->film[i].b = std::max(std::min(outputPtr[(i * 3) + 2], 255.f), 0.f);
		}
	}

	void render() {
		// General Render Function to Select Desired Rendering Method
		int renderMode = 3;
		if (renderMode == 0) renderSequential();
		if (renderMode == 1) renderMultithread();
		if (renderMode == 2) renderMultithreadDenoise();
		if (renderMode == 3) renderLightTrace();
		// if (renderMode == 4) renderInstantRadiosity();
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
				
				// Check for NaN and Inf Values
				Colour col = pathTrace(ray, samplers);
				if (std::isnan(col.r) || std::isnan(col.g) || std::isnan(col.b)) col = Colour(0.f, 0.f, 0.f);
				if (std::isinf(col.r) || std::isinf(col.g) || std::isinf(col.b)) col = Colour(0.f, 0.f, 0.f);

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
								
								// Check for NaN and Inf Values
								Colour col = pathTrace(ray, samplers);
								if (std::isnan(col.r) || std::isnan(col.g) || std::isnan(col.b)) col = Colour(0.f, 0.f, 0.f);
								if (std::isinf(col.r) || std::isinf(col.g) || std::isinf(col.b)) col = Colour(0.f, 0.f, 0.f);
								
								// film->splat(px, py, col);
								lightTrace(samplers);
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
								
								// Preparing the films for Denoising (AOVs)
								Colour normalCol = viewNormals(ray);
								normalFilm->splat(px, py, normalCol);
								Colour albedoCol = albedo(ray);
								albedoFilm->splat(px, py, albedoCol);

								// Check for NaN and Inf Values
								Colour pathCol = pathTrace(ray, samplers);
								if (std::isnan(pathCol.r) || std::isnan(pathCol.g) || std::isnan(pathCol.b)) continue;
								if (std::isinf(pathCol.r) || std::isinf(pathCol.g) || std::isinf(pathCol.b)) continue;
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

	// Light Tracing
	void renderLightTrace() {
		// Sequential Rendering
		film->incrementSPP();
		for (unsigned int y = 0; y < film->height; y++) {
			for (unsigned int x = 0; x < film->width; x++) {
				lightTrace(samplers);
			}
		}

		for (unsigned int y = 0; y < film->height; y++) {
			for (unsigned int x = 0; x < film->width; x++) {
				unsigned char r, g, b;
				film->tonemap(x, y, r, g, b, 2.f);
				canvas->draw(x, y, r, g, b);
			}
		}
	}

	// Instant Radiosity
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