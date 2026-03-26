#pragma once

#include "Core.h"
#include <random>
#include <algorithm>

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
		Vec3 vec = SphericalCoordinates::sphericalToWorld(theta, phi);
		return vec;
	}

	static float uniformHemispherePDF(const Vec3 wi) {
		// Add code here
		return 1.f / (2.f * M_PI);  // for now
	}

	static Vec3 cosineSampleHemisphere(float r1, float r2) {
		// Add code here
		float theta = acos(sqrtf(r1));
		float phi = 2.f * M_PI * r2;
		Vec3 vec = SphericalCoordinates::sphericalToWorld(theta, phi);
		return vec;
	}

	static float cosineHemispherePDF(const Vec3 wi) {
		// Add code here
		float theta = SphericalCoordinates::sphericalTheta(wi);
		return cos(theta) / M_PI;
	}

	static Vec3 uniformSampleSphere(float r1, float r2) {
		// Add code here
		float theta = acos(1.f - 2.f * r1);
		float phi = 2.f * M_PI * r2;
		Vec3 vec = SphericalCoordinates::sphericalToWorld(theta, phi);
		return vec;
	}

	static float uniformSpherePDF(const Vec3& wi) {
		// Add code here
		return 1.f / (4.f * M_PI);  // for now
	}
};