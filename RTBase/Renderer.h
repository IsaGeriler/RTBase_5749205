#pragma once

#include "Core.h"
#include "Sampling.h"
#include "Geometry.h"
#include "Imaging.h"
#include "Materials.h"
#include "Lights.h"
#include "Scene.h"
#include "GamesEngineeringBase.h"

#include <atomic>
#include <mutex>
#include <vector>
#include <thread>
#include <functional>

constexpr bool enable_mt = true;
constexpr bool enable_bvh = true;

struct ScreenTile {
	unsigned int tile_x{ 0 }, tile_y{ 0 };
	unsigned int tile_size{ 32 };
	std::atomic<bool> is_tile_rendered = false;

	// Get start index of x and y
	unsigned int tile_x_start() const { return std::max(static_cast<unsigned int>(0), tile_x); }
	unsigned int tile_y_start() const { return std::max(static_cast<unsigned int>(0), tile_y); }

	// Get end index of x and y
	unsigned int tile_x_end(Film* film) const { return std::min(tile_x + tile_size - 1, film->width - 1); }
	// unsigned int tile_x_end(Film* film) const { return std::min(tile_x + tile_size - 2, film->width - 1); }
	unsigned int tile_y_end(Film* film) const { return std::min(tile_y + tile_size - 1, film->height - 1); }
	// unsigned int tile_y_end(Film * film) const { return std::min(tile_y + tile_size - 2, film->height - 1); }
};

class RayTracer
{
public:
	Scene* scene;
	GamesEngineeringBase::Window* canvas;
	Film* film;
	MTRandom *samplers;
	std::thread **threads;
	int numProcs;
	void init(Scene* _scene, GamesEngineeringBase::Window* _canvas)
	{
		scene = _scene;
		canvas = _canvas;
		film = new Film();
		//film->init((unsigned int)scene->camera.width, (unsigned int)scene->camera.height, new BoxFilter());
		//film->init((unsigned int)scene->camera.width, (unsigned int)scene->camera.height, new TriangleFilter());
		//film->init((unsigned int)scene->camera.width, (unsigned int)scene->camera.height, new GaussianFilter(4.f, 0.5f));
		//film->init((unsigned int)scene->camera.width, (unsigned int)scene->camera.height, new MitchellNetravaliFilter());
		film->init((unsigned int)scene->camera.width, (unsigned int)scene->camera.height, new LanczosSincFilter(3.f, 3.f));
		SYSTEM_INFO sysInfo;
		GetSystemInfo(&sysInfo);
		numProcs = sysInfo.dwNumberOfProcessors;
		threads = new std::thread*[numProcs];
		samplers = new MTRandom[numProcs];
		clear();
	}
	void clear()
	{
		film->clear();
	}
	Colour computeDirect(ShadingData shadingData, Sampler* sampler)
	{
		// Is surface is specular we cannot computing direct lighting
		if (shadingData.bsdf->isPureSpecular() == true)
		{
			return Colour(0.0f, 0.0f, 0.0f);
		}

		// Compute direct lighting here
		float pmf, pdf;
		Light* light = scene->sampleLight(sampler, pmf);

		if (light->isArea()) {
			// Sample point on light and store returned emission
			Colour emittedColour;
			Vec3 samplePointOnLight = light->sample(shadingData, sampler, emittedColour, pdf);
			
			// wi - Direction to light
			Vec3 surfaceToLight = samplePointOnLight - shadingData.x;
			Vec3 wi = surfaceToLight.normalize();
			float cosineTermSurface = std::max(Dot(wi, shadingData.sNormal), 0.f);
			float cosineTermLight = std::max(-Dot(wi, light->normal(shadingData, wi)), 0.f);
			
			// Evaluate Visibility
			// V(xi <-> xi+1) - Binary Function, from Ray Trace
			if (scene->visible(shadingData.x, samplePointOnLight)) {
				// Evaluate Geometry Term
				// G(xi <-> xi+1) = [max((wi.n), 0) * max(-(wi.n'), 0) / lengthSq(xi - xi+1)] V(xi <-> xi+1)
				// n - normal at path vertex xi
				// n' - normal at path vertex xi+1
				float gTerm = (cosineTermSurface * cosineTermLight) / surfaceToLight.lengthSq();
				
				// Evaluate BSDF
				Colour BSDF = shadingData.bsdf->evaluate(shadingData, wi);

				// Multiply and return
				return BSDF * emittedColour * gTerm / (pmf * pdf);
			}
		}
	}
	Colour pathTrace(Ray& r, Colour& pathThroughput, int depth, Sampler* sampler)
	{
		// Add pathtracer code here
		return Colour(0.0f, 0.0f, 0.0f);
	}
	Colour direct(Ray& r, Sampler* sampler) {
		// Compute direct lighting for an image sampler here
		IntersectionData intersection = enable_bvh ? scene->bvh_traverse(r) : scene->traverse(r);
		ShadingData shadingData = scene->calculateShadingData(intersection, r);
		if (shadingData.t < FLT_MAX) {
			if (shadingData.bsdf->isLight()) {
				return shadingData.bsdf->emit(shadingData, shadingData.wo);
			}
			return computeDirect(shadingData, sampler);
		}
		return scene->background->evaluate(r.dir);
	}
	Colour albedo(Ray& r)
	{
		IntersectionData intersection = enable_bvh ? scene->bvh_traverse(r) : scene->traverse(r);
		ShadingData shadingData = scene->calculateShadingData(intersection, r);
		if (shadingData.t < FLT_MAX)
		{
			if (shadingData.bsdf->isLight())
			{
				return shadingData.bsdf->emit(shadingData, shadingData.wo);
			}
			return shadingData.bsdf->evaluate(shadingData, Vec3(0, 1, 0));
		}
		return scene->background->evaluate(r.dir);
	}
	Colour viewNormals(Ray& r)
	{
		IntersectionData intersection = enable_bvh ? scene->bvh_traverse(r) : scene->traverse(r);
		if (intersection.t < FLT_MAX)
		{
			ShadingData shadingData = scene->calculateShadingData(intersection, r);
			return Colour(fabsf(shadingData.sNormal.x), fabsf(shadingData.sNormal.y), fabsf(shadingData.sNormal.z));
		}
		return Colour(0.0f, 0.0f, 0.0f);
	}
	// Sequential vs Multithreaded Render
	void renderSequential(Film* film)
	{
		for (unsigned int y = 0; y < film->height; y++)
		{
			for (unsigned int x = 0; x < film->width; x++)
			{
				float px = x + 0.5f;
				float py = y + 0.5f;
				Ray ray = scene->camera.generateRay(px, py);
				//Colour col = viewNormals(ray);
				//Colour col = albedo(ray);
				Colour col = direct(ray, samplers);
				film->splat(px, py, col);
				unsigned char r = (unsigned char)(col.r * 255);
				unsigned char g = (unsigned char)(col.g * 255);
				unsigned char b = (unsigned char)(col.b * 255);
				
				// Select a Tonemap Operator to Apply:
				// 1 - Linear without gamma, 2 - Linear with gamma, 
				// 3 - Linear with gamma and exposure, 4 - Reinhard Global,
				// 5 - Filmic (Uncharted 2, Hable), 6 - ACES Filmic Curve,
				// 7 - Jim Hejl and Richard Burgess-Dawson
				film->tonemap(x, y, r, g, b, 7);
				canvas->draw(x, y, r, g, b);
			}
		}
	}
	void renderMultithread(Film* film)
	{
		std::mutex mtx;
		std::vector<std::thread> thread_pool;
		std::atomic<int> atomic_id_counter = 0;
		thread_pool.reserve(numProcs);

		unsigned int tile_size = 32;
		unsigned int tile_count = static_cast<unsigned int>(std::ceil(film->width / tile_size) * std::ceil(film->height / tile_size));

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

					if (!screen_tile.is_tile_rendered.load(std::memory_order_relaxed))
					{
						for (unsigned int y = screen_tile.tile_y_start(); y <= screen_tile.tile_y_end(film); ++y)
						{
							for (unsigned int x = screen_tile.tile_x_start(); x <= screen_tile.tile_x_end(film); ++x)
							{
								float px = x + 0.5f;
								float py = y + 0.5f;
								Ray ray = scene->camera.generateRay(px, py);
								//Colour col = viewNormals(ray);
								//Colour col = albedo(ray);
								Colour col = direct(ray, samplers);
								film->splat(px, py, col);
								unsigned char r = (unsigned char)(col.r * 255);
								unsigned char g = (unsigned char)(col.g * 255);
								unsigned char b = (unsigned char)(col.b * 255);

								// Select a Tonemap Operator to Apply:
								// 1 - Linear without gamma, 2 - Linear with gamma, 
								// 3 - Linear with gamma and exposure, 4 - Reinhard Global,
								// 5 - Filmic (Uncharted 2, Hable), 6 - ACES Filmic Curve,
								// 7 - Jim Hejl and Richard Burgess-Dawson
								film->tonemap(x, y, r, g, b, 7);
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
	void render()
	{
		film->incrementSPP();
		enable_mt ? renderMultithread(film) : renderSequential(film);
	}
	int getSPP()
	{
		return film->SPP;
	}
	void saveHDR(std::string filename)
	{
		film->save(filename);
	}
	void savePNG(std::string filename)
	{
		stbi_write_png(filename.c_str(), canvas->getWidth(), canvas->getHeight(), 3, canvas->getBackBuffer(), canvas->getWidth() * 3);
	}
};