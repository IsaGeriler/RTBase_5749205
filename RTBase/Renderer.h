#pragma once

#define MAX_VPLS 100

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
#include <vector>

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

// Virtual Point Lights
class VPL {
public:
	// Stored attributes
	ShadingData shadingData;
	Colour Le;
	
	// ShadingData created at each interaction when creating VPLS
	VPL(ShadingData _shadingData, Colour _Le) : shadingData(_shadingData), Le(_Le) {}
};

class RayTracer {
public:
	Scene* scene;
	GamesEngineeringBase::Window* canvas;
	MTRandom* samplers;
	HaltonSampler* haltonSamplers;  // For Instant Radiosity
	std::thread** threads;
	int numProcs;

	Film* film;
	Film* normalFilm;
	Film* albedoFilm;
	
	void init(Scene* _scene, GamesEngineeringBase::Window* _canvas) {
		scene = _scene;
		canvas = _canvas;

		// Creating separate films to store shading normals, albedos, and path trace splat colours for denoising (AOVs)
		film = new Film();
		film->init((unsigned int)scene->camera.width, (unsigned int)scene->camera.height, new BoxFilter());

		normalFilm = new Film();
		normalFilm->init((unsigned int)scene->camera.width, (unsigned int)scene->camera.height, new BoxFilter());

		albedoFilm = new Film();
		albedoFilm->init((unsigned int)scene->camera.width, (unsigned int)scene->camera.height, new BoxFilter());

		SYSTEM_INFO sysInfo;
		GetSystemInfo(&sysInfo);
		numProcs = sysInfo.dwNumberOfProcessors;
		threads = new std::thread * [numProcs];
		samplers = new MTRandom[numProcs];
		haltonSamplers = new HaltonSampler[numProcs];  // For Instant Radiosity
		
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
				if (pLight <= 0) return Colour(0.f, 0.f, 0.f);
				return emission * BSDF * gTerm * w / pLight;
			}
			return Colour(0.f, 0.f, 0.f);
		}
		// Environment Map
		else {
			// Sample from light, returns direction instead of point
			Vec3 wi = light->sample(shadingData, sampler, emission, pdf);
			if (pmf <= 0.f) return Colour(0.f, 0.f, 0.f);
			
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
				if (pLight <= 0) return Colour(0.f, 0.f, 0.f);
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
	
	Colour pathTraceRecursive(Ray& r, Colour& pathThroughput, int depth, Sampler* sampler, bool isSpecular = false) {
		// Add pathtracer code here
		// Trace Ray
		IntersectionData intersection = scene->traverse(r);
		ShadingData shadingData = scene->calculateShadingData(intersection, r);
		if (shadingData.t < FLT_MAX) {
			if (shadingData.bsdf->isLight()) {
				// Either emit at the first depth or specular surface, or use MIS to prevent double counting (need previous surface info.)
				return (depth == 0 || isSpecular) ? shadingData.bsdf->emit(shadingData, shadingData.wo) : Colour(0.f, 0.f, 0.f);
			}
			// Calculate Direct Lighting
			Colour direct = pathThroughput * computeDirect(shadingData, sampler);
			
			// Calculate Indirect Lighting
			float pdf;
			Colour indirect;
			Vec3 wi = shadingData.bsdf->sample(shadingData, sampler, indirect, pdf);
			Ray indirectRay(shadingData.x + (wi * EPSILON), wi);

			// Update path throughput (multiply with BSDF and cosine, divide by pdf)
			float cosine = Dot(wi, shadingData.sNormal);
			pathThroughput = pathThroughput * indirect * cosine / pdf;

			// Apply Russian Roulette
			if (depth >= 3) {
				// Normally rrp (Russian Roulette Probability) is std::min(pathThroughput.Lum(), 1.f)
				// However, I set to 0.99f as 1.f would somehow result in a stack overflow at my BVH Traverse code...
				float rrp = std::min(pathThroughput.Lum(), 0.99f);
				if (sampler->next() < rrp) pathThroughput = pathThroughput / rrp;
				else return direct;
			}
			// isSpecular will be the boolean state of the previous surface to contribute when the light is viewed in a specular surface
			return direct + pathTraceRecursive(indirectRay, pathThroughput, depth + 1, sampler, shadingData.bsdf->isPureSpecular());
		}
		return scene->background->evaluate(r.dir) * pathThroughput;
	}

	// --- Light Trace Start ---
	// Recheck... probably something is wrong here as only Cornell Box works properly
	void connectToCamera(Vec3 p, Vec3 n, Colour col) {
		// Handle connections to camera
		float x, y;
		// Check if p is on camera
		if (scene->camera.projectOntoCamera(p, x, y)) {
			// Compute geometry term between p and camera
			Vec3 cNormal = scene->camera.viewDirection;  // Camera normal
			Vec3 cPos = scene->camera.origin;			 // Camera position
			Vec3 cDirection = cPos - p;					 // Direction to camera

			// Visibility Check
			if (scene->visible(p, cPos)) {
				Vec3 wi = cDirection.normalize();
				if (cDirection.lengthSq() < EPSILON) return;

				// Need to compute We(x,w) = 1.f / (Afilm * cos4Theta)
				// Theta is angle between scene->camera.viewDirection and direction to camera
				float cosTheta = std::max(-Dot(cNormal, wi), EPSILON);
				float gTerm = cosTheta * std::max(Dot(n, wi), EPSILON) / std::max(cDirection.lengthSq(), EPSILON);
				float Afilm = scene->camera.Afilm;
				float W = 1.f / (Afilm * powf(cosTheta, 4));

				// Splat col onto film at coordinates from projectOntoCamera
				film->splat(x, y, col * gTerm * W);
			}
		}
	}
	
	void lightTrace(Sampler* sampler) {
		// Handles starting a light path
		// Sample a light source
		float pmf;
		Light* light = scene->sampleWeightedLight(sampler, pmf);
		if (pmf <= 0.f) return;
		Colour pathThroughput(1.f, 1.f, 1.f);

		// Area Light
		if (light->isArea()) {
			// Sample point and direction from on light source
			float pdfDirection, pdfPosition;
			Vec3 p = light->samplePositionFromLight(sampler, pdfPosition);
			Vec3 wi = light->sampleDirectionFromLight(sampler, pdfDirection);
			if (pdfPosition <= 0.f) return;

			// Light Normal
			Vec3 n = light->normal(ShadingData(), p);
			
			// Connect position to camera
			Colour col = light->evaluate(-wi) / (pdfPosition * pmf);
			connectToCamera(p, n, col);
			
			// Create a ray starting at p in direction wi and then call lightTracePath
			Ray r(p + (wi * EPSILON), wi);
			lightTracePath(r, pathThroughput, col, sampler, 0);
		}
		// Environment Map
		else {
			// Need to flip direction to trace into scene (done inside the function already)
			float pdfBounds, pdfDirection;
			Vec3 wi = light->sampleDirectionFromLight(sampler, pdfDirection);
			
			// Sample position on the scene
			Vec3 sampledPos = light->samplePositionFromLight(sampler, pdfBounds);

			// Light Normal
			Vec3 n = light->normal(ShadingData(), sampledPos);
			
			// Overall PDF is product of Direction PDF and Position on bounds PDF
			float pdf = pdfBounds + pdfDirection;

			// Connect position to camera
			Colour col = light->evaluate(-wi) / pdf * pmf;
			connectToCamera(sampledPos, n, col);
			
			// Create a ray starting at p in direction wi and then call lightTracePath
			Ray r(sampledPos + (wi * EPSILON), wi);
			lightTracePath(r, pathThroughput, col, sampler, 0);
		}
	}

	void lightTracePath(Ray& r, Colour pathThroughput, Colour Le, Sampler* sampler, int depth) {
		// Handles tracing the rest of the light path
		IntersectionData intersection = scene->traverse(r);
		ShadingData shadingData = scene->calculateShadingData(intersection, r);
		if (shadingData.t < FLT_MAX) {
			if (shadingData.bsdf->isLight()) return;
			if (depth >= 10) return;

			// Use direction to camera (need to normalize)
			Vec3 dirToCamera = scene->camera.origin - shadingData.x;
			dirToCamera.normalize();

			// col has value of pathThroughput * shadingData.bsdf->evaluate(shadingData , wi) * Le
			Colour col = pathThroughput * shadingData.bsdf->evaluate(shadingData, dirToCamera) * Le;
			
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

			float corrFactor1 = fabs(Dot(shadingData.wo, shadingData.sNormal)) / fabs(Dot(shadingData.wo, shadingData.gNormal));
			float corrFactor2 = fabs(Dot(dirToCamera, shadingData.gNormal)) / fabs(Dot(dirToCamera, shadingData.sNormal));
			float correction = corrFactor1 * corrFactor2;
			
			// Connect each intersection to camera
			if (!shadingData.bsdf->isPureSpecular()) connectToCamera(shadingData.x, shadingData.sNormal, pathThroughput * col * correction);
			lightTracePath(indirectRay, pathThroughput, Le, sampler, depth + 1);
		}
	}
	// --- Light Trace End ---

	// --- Instant Radiosity Start ---
	// Vector for storing VPL
	std::vector<VPL> VPLs;

	Colour instantRadiosity(Ray& r, HaltonSampler* sampler) {
		IntersectionData intersection = scene->traverse(r);
		ShadingData shadingData = scene->calculateShadingData(intersection, r);
		if (shadingData.t < FLT_MAX) {
			if (shadingData.bsdf->isLight()) {
				return shadingData.bsdf->emit(shadingData, shadingData.wo);
			}
			// Compute Direct
			Colour direct = computeDirectVPL(shadingData, sampler);
			sampler->reset();

			// Compute VPL Contribution
			Colour vplContribution = contributeVPL(shadingData);

			// BSDF = Direct + Contribution
			return direct + vplContribution;
		}
		return scene->background->evaluate(r.dir);
	}
	
	// Handle VPL Contribution
	Colour contributeVPL(ShadingData& shadingData) {
		Colour contribution;
		for (unsigned int i = 0; i < VPLs.size(); i++) {
			Vec3 wi = (VPLs[i].shadingData.x - shadingData.x).normalize();
			Vec3 wiVPL = -wi;

			if (scene->visible(shadingData.x, VPLs[i].shadingData.x)) {
				// GTerm
				float gTerm = std::max(Dot(shadingData.sNormal, wi), EPSILON) * std::max(Dot(VPLs[i].shadingData.sNormal, wiVPL), EPSILON) / std::max((VPLs[i].shadingData.x - shadingData.x).lengthSq(), 1.f);

				// BSDF for VPL Contribution
				Colour BSDF = shadingData.bsdf->evaluate(shadingData, wi);
				Colour vplBSDF = VPLs[i].shadingData.bsdf->evaluate(VPLs[i].shadingData, wiVPL);
				contribution = contribution + (BSDF * vplBSDF * VPLs[i].Le * gTerm);
			}
		}
		return contribution;
	}

	// Handle VPL Direct Lighting by using Halton Sampler (Area Light Only)
	Colour computeDirectVPL(ShadingData& shadingData, HaltonSampler* sampler) {
		// If surface is specular we cannot computing direct lighting
		if (shadingData.bsdf->isPureSpecular() == true) {
			return Colour(0.0f, 0.0f, 0.0f);
		}
		// Compute direct lighting here
		// Sample a light
		float pdf, pmf;
		Colour emission;
		Light* light = scene->sampleLight(sampler, pmf);

		// Area Light Only
		if (light->isArea()) {
			// Sample point on light and store returned emission
			Vec3 samplePointOnLight = light->sample(shadingData, sampler, emission, pdf);
			if (pmf <= 0.f) return Colour(0.f, 0.f, 0.f);

			// Calculate Geometry Term
			Vec3 surfaceToLight = samplePointOnLight - shadingData.x;
			Vec3 wi = surfaceToLight.normalize();
			// if (surfaceToLight.lengthSq() < 0.1) return Colour(0.f, 0.f, 0.f);
			float gTerm = (std::max(Dot(wi, shadingData.sNormal), EPSILON) * std::max(-Dot(wi, light->normal(shadingData, wi)), EPSILON)) / std::max(surfaceToLight.lengthSq(), 1.f);
			
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
	}
	
	void traceVPL(HaltonSampler* sampler) {
		VPLs.clear();
		for (unsigned int i = 0; i < MAX_VPLS; i++) {
			// Sample a light source
			float pmf;
			Light* light = scene->sampleWeightedLight(sampler, pmf);
			Colour pathThroughput(1.f, 1.f, 1.f);

			// Area Light
			if (light->isArea()) {
				// Sample point and direction from on light source
				float pdfDirection, pdfPosition;
				Vec3 p = light->samplePositionFromLight(sampler, pdfPosition);
				Vec3 wi = light->sampleDirectionFromLight(sampler, pdfDirection);

				// Evaluate colour from direction
				Colour col = light->evaluate(-wi) / (pdfPosition * pdfDirection * pmf * MAX_VPLS);

				// Create a ray starting at p in direction wi and then call lightTracePath
				Ray r(p + (wi * EPSILON), wi);
				sampler->reset();
				traceVPLRecursive(r, pathThroughput, col, sampler, 0);
			}
			sampler->reset();
		}
	}
	
	void traceVPLRecursive(Ray& r, Colour pathThroughput, Colour Le, HaltonSampler* sampler, int depth, bool isSpecular = false) {
		IntersectionData intersection = scene->traverse(r);
		ShadingData shadingData = scene->calculateShadingData(intersection, r);
		if (shadingData.t < FLT_MAX) {
			if (shadingData.bsdf->isLight()) { if (depth == 0 || isSpecular) return; }
			// Store a VPL
			VPL vpl(shadingData, pathThroughput * Le);
			VPLs.emplace_back(vpl);

			// IndirectBSDF
			float pdf;
			Colour indirect;
			Vec3 wi = shadingData.bsdf->sample(shadingData, sampler, indirect, pdf);
			Ray indirectRay(shadingData.x + (wi * EPSILON), wi);

			// Update path throughput (multiply with IndirectBSDF and cosine, divide by pdf)
			float cosine = std::max(Dot(wi, shadingData.sNormal), EPSILON);
			pathThroughput = pathThroughput * indirect * cosine / pdf;

			// Apply Russian Roulette
			if (depth >= 3) {
				float rrp = std::min(pathThroughput.Lum(), 0.95f);
				if (sampler->next() < rrp) {
					pathThroughput = pathThroughput / rrp;
				}
				else return;
			}
			sampler->reset();
			traceVPLRecursive(indirectRay, pathThroughput, Le, sampler, depth + 1, shadingData.bsdf->isPureSpecular());
		}
	}
	// --- Instant Radiosity End ---

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
		unsigned int width = film->width;
		unsigned int height = film->height;
		unsigned int pixels = width * height;

		// OIDN
		oidn::DeviceRef device = oidn::newDevice(oidn::DeviceType::CPU);
		device.commit();

		// Check for errors
		const char* errorMessage;
		if (device.getError(errorMessage) != oidn::Error::None) std::cout << "Error: " << errorMessage << std::endl;

		oidn::BufferRef colourBuf = device.newBuffer(width * height * 3 * sizeof(float));
		oidn::BufferRef albedoBuf = device.newBuffer(width * height * 3 * sizeof(float));
		oidn::BufferRef normalBuf = device.newBuffer(width * height * 3 * sizeof(float));
		oidn::BufferRef outputBuf = device.newBuffer(width * height * 3 * sizeof(float));

		oidn::FilterRef filter = device.newFilter("RT");
		filter.setImage("color", colourBuf, oidn::Format::Float3, width, height);   // beauty
		filter.setImage("albedo", albedoBuf, oidn::Format::Float3, width, height);  // auxilary
		filter.setImage("normal", normalBuf, oidn::Format::Float3, width, height);  // auxilary
		filter.setImage("output", outputBuf, oidn::Format::Float3, width, height);  // denoised beauty
		filter.set("hdr", true);
		filter.commit();

		float* albedoPtr = (float*)albedoBuf.getData();
		for (unsigned int i = 0; i < pixels; i++) {
			albedoPtr[(i * 3)] = albedoFilm->film[i].r;
			albedoPtr[(i * 3) + 1] = albedoFilm->film[i].g;
			albedoPtr[(i * 3) + 2] = albedoFilm->film[i].b;
		}

		float* normalPtr = (float*)normalBuf.getData();
		for (unsigned int i = 0; i < pixels; i++) {
			normalPtr[(i * 3)] = normalFilm->film[i].r;
			normalPtr[(i * 3) + 1] = normalFilm->film[i].g;
			normalPtr[(i * 3) + 2] = normalFilm->film[i].b;
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
			film->film[i].r = outputPtr[(i * 3)];
			film->film[i].g = outputPtr[(i * 3) + 1];
			film->film[i].b = outputPtr[(i * 3) + 2];
		}
	}

	void render() {
		// General Render Function to Select Desired Rendering Method
		int renderMode = 3;
		if (renderMode == 0) renderSequential();
		if (renderMode == 1) renderMultithread();
		if (renderMode == 2) renderMultithreadDenoise();
		if (renderMode == 3) renderLightTrace();
		if (renderMode == 4) renderInstantRadiositySequential();
		if (renderMode == 5) renderInstantRadiosityMultithread();
	}

	void renderSequential() {
		// Sequential Rendering
		film->incrementSPP();
		for (unsigned int y = 0; y < film->height; y++) {
			for (unsigned int x = 0; x < film->width; x++) {
				float px = x + samplers->next();  // + 0.5f;
				float py = y + samplers->next();  // + 0.5f;
				Ray ray = scene->camera.generateRay(px, py);
				
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
								
								// Check for NaN and Inf Values
								Colour col = pathTrace(ray, samplers);
								if (std::isnan(col.r) || std::isnan(col.g) || std::isnan(col.b)) col = Colour(0.f, 0.f, 0.f);
								if (std::isinf(col.r) || std::isinf(col.g) || std::isinf(col.b)) col = Colour(0.f, 0.f, 0.f);
								if (col.Lum() > 10.f) col = col * (10.f / col.Lum());

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

	void renderInstantRadiositySequential() {
		// Sequential Rendering
		film->incrementSPP();

		// Reset Halton Sampler
		haltonSamplers->hardReset();

		// Trace and Generate VPLs for Instant Radiosity
		traceVPL(haltonSamplers);

		// Reset Halton Sampler (for pixel calculations)
		haltonSamplers->hardReset();
		for (unsigned int y = 0; y < film->height; y++) {
			for (unsigned int x = 0; x < film->width; x++) {
				// Using Mersenne Twister for plotting pixels, as Halton Sequence causes either aliasing
				// or patterns of black pixels, caused by Halton Sequence
				float px = x + samplers->next();
				float py = y + samplers->next();
				Ray ray = scene->camera.generateRay(px, py);

				// Check for NaN and Inf Values
				Colour col = instantRadiosity(ray, haltonSamplers);
				haltonSamplers->reset();

				if (std::isnan(col.r) || std::isnan(col.g) || std::isnan(col.b)) col = Colour(0.f, 0.f, 0.f);
				if (std::isinf(col.r) || std::isinf(col.g) || std::isinf(col.b)) col = Colour(0.f, 0.f, 0.f);

				film->splat(px, py, col);
				unsigned char r, g, b;
				film->tonemap(x, y, r, g, b, 2.f);
				canvas->draw(x, y, r, g, b);
			}
		}
	}

	void renderInstantRadiosityMultithread() {
		// Multithreaded Rendering
		film->incrementSPP();
		std::mutex mtx;
		std::vector<std::thread> thread_pool;
		std::atomic<int> atomic_id_counter = 0;
		thread_pool.reserve(numProcs);
		
		// Reset Halton Sampler
		haltonSamplers->hardReset();

		// Trace and Generate VPLs for Instant Radiosity
		traceVPL(haltonSamplers);

		// Reset Halton Sampler (for pixel calculations)
		haltonSamplers->hardReset();

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

								// Check for NaN and Inf Values
								Colour col = instantRadiosity(ray, haltonSamplers);
								haltonSamplers->reset();
								if (std::isnan(col.r) || std::isnan(col.g) || std::isnan(col.b)) col = Colour(0.f, 0.f, 0.f);
								if (std::isinf(col.r) || std::isinf(col.g) || std::isinf(col.b)) col = Colour(0.f, 0.f, 0.f);
								if (col.Lum() > 10.f) col = col * (10.f / col.Lum());

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