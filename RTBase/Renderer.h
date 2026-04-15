#pragma once

#include "Core.h"
#include "Sampling.h"
#include "Geometry.h"
#include "Imaging.h"
#include "Materials.h"
#include "Lights.h"
#include "Scene.h"
#include "GamesEngineeringBase.h"
#include <thread>
#include <functional>
#include <mutex>

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
	Film* film;
	MTRandom* samplers;
	std::thread** threads;
	int numProcs;
	
	void init(Scene* _scene, GamesEngineeringBase::Window* _canvas) {
		scene = _scene;
		canvas = _canvas;
		film = new Film();
		// film->init((unsigned int)scene->camera.width, (unsigned int)scene->camera.height, new BoxFilter());
		// film->init((unsigned int)scene->camera.width, (unsigned int)scene->camera.height, new GaussianFilter(2.f, 0.5f));
		film->init((unsigned int)scene->camera.width, (unsigned int)scene->camera.height, new MitchellNetravali());
		SYSTEM_INFO sysInfo;
		GetSystemInfo(&sysInfo);
		numProcs = sysInfo.dwNumberOfProcessors;
		threads = new std::thread * [numProcs];
		samplers = new MTRandom[numProcs];
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
		float pmf;
		Light* light = scene->sampleLight(sampler, pmf);

		// Check if light is area or environment light
		if (light->isArea()) {
			// Sample point on light and store returned emission
			float pdf;
			Colour emission;
			Vec3 samplePointOnLight = light->sample(shadingData, sampler, emission, pdf);

			// Calculate Geometry Term
			Vec3 surfaceToLight = samplePointOnLight - shadingData.x;
			Vec3 wi = surfaceToLight.normalize();
			float gTerm = (std::max(Dot(wi, shadingData.sNormal), 0.f) * std::max(-Dot(wi, light->normal(shadingData, wi)), 0.f)) / surfaceToLight.lengthSq();

			// Calculate Visibility
			// V[x(i) <-> x(i+1)] - Binary function, from Ray Tracing
			if (scene->visible(shadingData.x, samplePointOnLight)) {
				Colour BSDF = shadingData.bsdf->evaluate(shadingData, wi);
				return emission * BSDF * gTerm / (pdf * pmf);
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
			if (depth > 10) return direct;

			// Sample Indirect Direction
			// Vec3 incomingRadiance = SamplingDistributions::cosineSampleHemisphere(sampler->next(), sampler->next());
			// float pdf = SamplingDistributions::cosineHemispherePDF(incomingRadiance);
			// incomingRadiance = shadingData.frame.toWorld(incomingRadiance);
			// Colour indirect = shadingData.bsdf->evaluate(shadingData, wi);
			
			Colour indirect;
			float pdf;
			Vec3 wi = shadingData.bsdf->sample(shadingData, sampler, indirect, pdf);
			r.init(shadingData.x + (wi * EPSILON), wi);

			// Update path throughput (multiply with BSDF and cosine)
			float cosine = std::max(Dot(wi, shadingData.sNormal), 0.f);
			pathThroughput = pathThroughput * indirect * cosine / pdf;

			// Apply Russian Roulette
			if (depth > 3) {
				float rrp = std::min(pathThroughput.Lum(), 1.f);
				if (sampler->next() < rrp) {
					pathThroughput = pathThroughput / rrp;
					return direct + pathTraceRecursive(r, pathThroughput, depth + 1, sampler);
				}
				else return direct;
			}
			return direct + pathTraceRecursive(r, pathThroughput, depth + 1, sampler);
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

	void render() {
		film->incrementSPP();

		std::mutex mtx;
		std::vector<std::thread> thread_pool;
		std::atomic<int> atomic_id_counter = 0;
		thread_pool.reserve(numProcs);

		unsigned int tile_size = 32;
		unsigned int tile_count = (unsigned int)(std::ceil(film->width / tile_size) * std::ceil(film->height / tile_size));

		for (unsigned int i = 0; i < numProcs; ++i) {
			thread_pool.emplace_back([&]() {
				// Work function
				ScreenTile screen_tile;
				unsigned int tile_id = 0;

				while ((tile_id = atomic_id_counter.fetch_add(1)) < tile_count) {
					// Initialize ScreenTile structure's attributes
					{
						std::lock_guard<std::mutex> lock(mtx);
						screen_tile.tile_x = (tile_id % (unsigned int)std::ceil(film->width / tile_size)) * tile_size;
						screen_tile.tile_y = (tile_id / (unsigned int)std::ceil(film->width / tile_size)) * tile_size;
						screen_tile.tile_size = tile_size;
						screen_tile.is_tile_rendered = false;
					}

					if (!screen_tile.is_tile_rendered.load(std::memory_order_relaxed)) {
						for (unsigned int y = screen_tile.tile_y_start(); y <= screen_tile.tile_y_end(film); ++y) {
							for (unsigned int x = screen_tile.tile_x_start(); x <= screen_tile.tile_x_end(film); ++x) {
								//float px = x + 0.5f;
								//float py = y + 0.5f;
								// Path Trace Update
								float px = x + samplers->next();
								float py = y + samplers->next();
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