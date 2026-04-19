#pragma once

#include <algorithm>
#include <random>

#include "Core.h"

class Sampler {
public:
	virtual float next() = 0;
};

class MTRandom : public Sampler {
public:
	std::mt19937 generator;
	std::uniform_real_distribution<float> dist;

	MTRandom(unsigned int seed = 1) : dist(0.0f, 1.0f) {
		generator.seed(seed);
	}
	
	float next() {
		return dist(generator);
	}
};

// Note all of these distributions assume z-up coordinate system
class SamplingDistributions {
public:
	static Vec3 uniformSampleHemisphere(float r1, float r2) {
		// Add code here
		float theta = acos(r1);
		float phi = 2.f * M_PI * r2;
		return SphericalCoordinates::sphericalToWorld(theta, phi);
	}

	static float uniformHemispherePDF(const Vec3 wi) {
		// Add code here
		// Solid Angle PDF
		return 1.f / (2.f * M_PI);
	}
	
	static Vec3 cosineSampleHemisphere(float r1, float r2) {
		// Add code here
		float theta = acos(sqrtf(r1));
		float phi = 2.f * M_PI * r2;
		return SphericalCoordinates::sphericalToWorld(theta, phi);
	}

	static float cosineHemispherePDF(const Vec3 wi) {
		// Add code here
		float theta = SphericalCoordinates::sphericalTheta(wi);
		return cos(theta) / M_PI;
	}

	static Vec3 uniformSampleSphere(float r1, float r2) {
		// Add code here
		float theta = acos(1.f - (2.f * r1));
		float phi = 2.f * M_PI * r2;
		return SphericalCoordinates::sphericalToWorld(theta, phi);
	}

	static float uniformSpherePDF(const Vec3& wi) {
		// Add code here
		// Solid Angle PDF
		return 1.f / (4.f * M_PI);
	}
};

// Multidimensional Distribution: Tabulated Distribution
class TabulatedDistribution {
	static Vec3 sample() {
		// Sample CDF(v) via binary search
		// Sample u from CDF(u|v)
	}
	
	static float pdf(const Vec3 v) {
		// PDF = (F[ui', vj'] / (1/NwNh) (nested loop over w and h) F[ui, vj])
	}
};