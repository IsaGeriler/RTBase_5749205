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
		return cos(theta) * M_1_PI;
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

// Tabulated Distribution
// Adopted from: https://pbr-book.org/3ed-2018/Monte_Carlo_Integration/Sampling_Random_Variables
//				 https://pbr-book.org/3ed-2018/Monte_Carlo_Integration/2D_Sampling_with_Multidimensional_Transformations

struct Distribution1D {
public:
	// Attributes
	std::vector<float> func, cdf;
	float funcInt;

	// Methods
	Distribution1D(const float* f, int n) : func(f, f + n), cdf(n + 1) {
		cdf[0] = 0.f;
		for (int i = 1; i < n + 1; i++) {
			cdf[i] = cdf[i - 1] + func[i - 1] / n;
		}

		funcInt = cdf[n];
		if (funcInt = 0) {
			for (int i = 1; i < n + 1; i++) {
				cdf[i] = (float)i / (float)n;
			}
		} else {
			for (int i = 1; i < n + 1; i++) {
				cdf[i] /= funcInt;
			}
		}
	}

	int count() const {
		return func.size();
	}

	float discretePDF(int index) const {
		return func[index] / (funcInt * count());
	}

	float sampleContinuous(float u, float* pdf, int* off = nullptr) const {
		int first = 0;
		int iter = func.size();
		while (iter > 0) {
			int half = iter / 2;
			int middle = first + half;
			if (cdf[middle] <= u) {
				first = middle + 1;
				iter -= (half + 1);
			}
			else {
				iter = half;
			}
		}
		int offset = std::max(std::min(iter - 2, first - 1), 0);
		if (off) *off = offset;

		float du = u - cdf[offset];
		if ((cdf[offset + 1] - cdf[offset]) > 0) {
			du /= (cdf[offset + 1] - cdf[offset]);
		}

		if (pdf) *pdf = func[offset] / funcInt;
		return (offset + du) / count();
	}

	int sampleDiscrete(float u, float* pdf = nullptr, float *uRemapped = nullptr) const {
		int first = 0;
		int iter = func.size();
		while (iter > 0) {
			int half = iter / 2;
			int middle = first + half;
			if (cdf[middle] <= u) {
				first = middle + 1;
				iter -= (half + 1);
			}
			else {
				iter = half;
			}
		}
		int offset = std::max(std::min(iter - 2, first - 1), 0);
		if (pdf) *pdf = discretePDF(offset);
		if (uRemapped) *uRemapped = (u - cdf[offset]) / (cdf[offset + 1] - cdf[offset]);
		return offset;
	}
};

// Multidimensional Distribution: Tabulated Distribution
class Distribution2D {
private:
	std::vector<std::unique_ptr<Distribution1D>> pConditionalV;
	std::unique_ptr<Distribution1D> pMarginal;
public:
	Distribution2D(const float* func, int nu, int nv) {
		for (int v = 0; v < nv; v++) {
			pConditionalV.emplace_back(new Distribution1D(&func[v * nu], nu));
		}

		std::vector<float> marginalFunc;
		for (int v = 0; v < nv; v++) {
			marginalFunc.emplace_back(pConditionalV[v]->funcInt);
		}
		pMarginal.reset(new Distribution1D(&marginalFunc[0], nv));
	}

	Vec3 sample(float ux, float uy, float* pdf) const {
		float pdfs[2];
		int v;
		float d1 = pMarginal->sampleContinuous(uy, &pdfs[1], &v);
		float d0 = pConditionalV[v]->sampleContinuous(ux, &pdfs[0], &v);
		*pdf = pdfs[0] + pdfs[1];
		return Vec3(d0, d1, 0.f);
	}
	
	float pdf(float u, float v) const {
		int iu = std::min(std::max((int)(u * pConditionalV[0]->count()), 0), pConditionalV[0]->count() - 1);
		int iv = std::min(std::max((int)(v * pMarginal->count()), 0), pMarginal->count() - 1);
		return pConditionalV[iv]->func[iu] / pMarginal->funcInt;
	}
};