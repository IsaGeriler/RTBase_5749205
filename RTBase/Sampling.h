#pragma once

#include <algorithm>
#include <memory>
#include <random>
#include <vector>

#include "Core.h"
#include "Imaging.h"

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
		return (cos(theta) <= 0.f) ? 0.f : (cos(theta) * M_1_PI);
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

// Tabulated Distribution for Environment Mapping
// Adopted from: https://pbr-book.org/3ed-2018/Monte_Carlo_Integration/Sampling_Random_Variables
//				 https://pbr-book.org/3ed-2018/Monte_Carlo_Integration/2D_Sampling_with_Multidimensional_Transformations
class Distribution1D {
public:
	// Attributes
	std::vector<float> func, cdf;
	float funcInt;

	// Constructor
	Distribution1D(std::vector<float>& f, int n) : func(f), cdf(n + 1) {
		// Compute integral of step function at xi
		cdf[0] = 0.f;
		for (int i = 1; i < n + 1; ++i) {
			cdf[i] = cdf[i - 1] + func[i - 1] / n;
		}

		// Transform into CDF
		funcInt = cdf[n];
		if (funcInt == 0) {
			for (int i = 1; i < n + 1; ++i) {
				cdf[i] = (float)i / (float)n;
			}
		} else {
			for (int i = 1; i < n + 1; ++i) {
				cdf[i] /= funcInt;
			}
		}
	}

	// Methods
	int count() const { return func.size(); }
	float discretePDF(int index) const { return func[index] / (funcInt * count()); }

	int sampleDiscrete(float u, float& pdf) const {
		// Find surrounding CDF segments and offset
		int first = 0;
		int size = cdf.size();
		int iter = size;
		// Binary Search
		while (iter > 0) {
			int half = iter / 2;
			int middle = first + half;
			if (cdf[middle] <= u) {
				first = middle + 1;
				iter -= (half + 1);
			}
			else iter = half;
		}
		int offset = std::max(std::min(size - 2, first - 1), 0);
		pdf = discretePDF(offset);
		return offset;
	}
};

class Distribution2D {
public:
	// Attributes
	std::vector<Distribution1D*> pConditionalV;
	std::unique_ptr<Distribution1D> pMarginal;

	// Constructor
	Distribution2D(Texture* env) {
		int width = env->width;
		int height = env->height;
		std::vector<float> marginalFunc(height);
		pConditionalV.resize(width);
		
		for (int v = 0; v < height; v++) {
			std::vector<float> F;
			for (int u = 0; u < width; u++) {
				// Incorporate sin into Luminance weights to approximately cancel in denominator!
				float luminance = env->texels[(v * width) + u].Lum();
				// Normalize, adding with 0.5 because v starts at 0 (and will over-shoot whites)
				float sinTheta = sin(M_PI * ((v + 0.5f) / height));
				F.emplace_back(luminance * sinTheta);
			}
			pConditionalV[v] = new Distribution1D(F, width);
			marginalFunc[v] = pConditionalV[v]->funcInt;
		}
		pMarginal.reset(new Distribution1D(marginalFunc, height));
	}

	// Methods
	// In Distribution1D, we sample from a discrete set
	// However, Distribution2D contains continuous set
	// Have to convert our pdfs accordingly
	void sample(float ru, float rv, float& u, float& v, float& pdf) const {
		float pdfu, pdfv;
		int dv = pMarginal->sampleDiscrete(rv, pdfv);
		int du = pConditionalV[dv]->sampleDiscrete(ru, pdfu);
		
		int width = pConditionalV[dv]->count();
		int height = pMarginal->count();

		// Normalize (u, v) in [0, 1] range
		// If we don't sum up with the sampled random numbers,
		// the lighting will over-shoot white colour/light only
		u = (du + ru) / width;
		v = (dv + rv) / height;
		pdf = (pdfu * width) * (pdfv * height);
	}

	float pdf(float u, float v) const {
		int iu = std::min(std::max((int)(u * pConditionalV[0]->count()), 0), pConditionalV[0]->count() - 1);
		int iv = std::min(std::max((int)(v * pMarginal->count()), 0), pMarginal->count() - 1);
		return pConditionalV[iv]->func[iu] / pMarginal->funcInt;
	}
};