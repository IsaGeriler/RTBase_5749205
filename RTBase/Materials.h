#pragma once

#include "Core.h"
#include "Imaging.h"
#include "Sampling.h"
#include <algorithm>

#pragma warning( disable : 4244)
#pragma warning( disable : 4305)  // Double to float

class BSDF;

class ShadingData {
public:
	Vec3 x;
	Vec3 wo;
	Vec3 sNormal;
	Vec3 gNormal;

	float tu;
	float tv;
	
	Frame frame;
	BSDF* bsdf;
	
	float t;

	ShadingData() {}
	ShadingData(Vec3 _x, Vec3 n) {
		x = _x;
		gNormal = n;
		sNormal = n;
		bsdf = NULL;
	}
};

class ShadingHelper {
public:
	// Done 100% Sure
	static float fresnelDielectric(float cosTheta, float iorInt, float iorExt) {
		// Find eta (depends on direction/shadingData.wo's sign)
		float cosTheta_i = std::max(std::min(cosTheta, 1.f), -1.f);
		float eta = iorExt / iorInt;
		if (cosTheta_i <= 0.f) {
			eta = 1.f / eta;
			cosTheta_i = fabs(cosTheta_i);
		}

		// Given cosTheta_i, calculate cosTheta_t from Snell's Law
		float sinTheta_i = sqrtf(std::max(1.f - powf(cosTheta_i, 2), 0.f));
		float sinTheta_t = eta * sinTheta_i;

		// What if sinTheta_t is greater or equal than one - Total Internal Reflection
		if (sinTheta_t >= 1.f) return 1.f;
		float cosTheta_t = sqrtf(std::max(1.f - powf(sinTheta_t, 2), 0.f));
		
		// Return the squared average of perpendicular and parallel
		float fParallel = (cosTheta_i - eta * cosTheta_t) / (cosTheta_i + eta * cosTheta_t);
		float fPerpendicular = (eta * cosTheta_i - cosTheta_t) / (eta * cosTheta_i + cosTheta_t);
		return (powf(fParallel, 2) + powf(fPerpendicular, 2)) * 0.5f;
	}

	// Done I guess... not so sure
	static Colour fresnelConductor(float cosTheta, Colour ior, Colour k) {
		float fCosThetaI = fabs(cosTheta);
		float fSinThetaI = sqrtf(std::max(1.f - powf(fCosThetaI, 2), 0.f));
		Colour cosThetaI(fCosThetaI, fCosThetaI, fCosThetaI);
		Colour sinThetaI(fSinThetaI, fSinThetaI, fSinThetaI);

		// Return the squared average of perpendicular and parallel
		Colour parallelSquared = (
			(((ior * ior) + (k * k)) * (cosThetaI * cosThetaI)) - (ior * 2 * cosThetaI) + (sinThetaI * sinThetaI) /
			(((ior * ior) + (k * k)) * (cosThetaI * cosThetaI)) + (ior * 2 * cosThetaI) + (sinThetaI * sinThetaI)
		);
		Colour perpendicularSquared = (
			(ior * ior) + (k * k) - (ior * 2 * cosThetaI) + (cosThetaI * cosThetaI) /
			(ior * ior) + (k * k) + (ior * 2 * cosThetaI) + (cosThetaI * cosThetaI)
		);
		return (parallelSquared + perpendicularSquared) * 0.5f;
	}

	// Done 100% Sure
	static float lambdaGGX(Vec3 wi, float alpha) {
		// Isotropic Lambda Function for GGX (Trowbridge-Reitz)
		float cosTheta = wi.z;
		float sinTheta = sqrtf(std::max(1.f - powf(cosTheta, 2), 0.f));
		float tanTheta = std::fabs(sinTheta / cosTheta);
		if (std::isinf(tanTheta)) return 0.f;
		return (sqrtf(1.f + (powf(alpha, 2) * powf(tanTheta, 2))) - 1.f) * 0.5f;
	}

	// Done 100% Sure
	static float Gggx(Vec3 wi, Vec3 wo, float alpha) {
		// Assume masking and shadowing are statistically independent
		// G(wo,wi) = G1(wo,wm) G1(wi,wm)
		// For GGX (Trowbridge-Reitz) = 1.f / (1.f + Λ(wo) + Λ(wi))
		return 1.f / (1.f + lambdaGGX(wo, alpha) + lambdaGGX(wi, alpha));
	}
	
	// Done 100% Sure
	static float Dggx(Vec3 h, float alpha) {
		// Isotropic Distribution for GGX (Trowbridge-Reitz)
		return powf(alpha, 2) / (M_PI * powf(powf(h.z, 2) * (powf(alpha, 2) - 1.f) + 1.f, 2));
	}
};

class BSDF {
public:
	Colour emission;
	virtual Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf) = 0;
	virtual Colour evaluate(const ShadingData& shadingData, const Vec3& wi) = 0;
	virtual float PDF(const ShadingData& shadingData, const Vec3& wi) = 0;
	virtual bool isPureSpecular() = 0;
	virtual bool isTwoSided() = 0;
	
	bool isLight() {
		return emission.Lum() > 0 ? true : false;
	}

	void addLight(Colour _emission) {
		emission = _emission;
	}

	Colour emit(const ShadingData& shadingData, const Vec3& wi) {
		return emission;
	}

	virtual float mask(const ShadingData& shadingData) = 0;
};

// Done 100% Sure
class DiffuseBSDF : public BSDF {
public:
	Texture* albedo;

	DiffuseBSDF() = default;
	DiffuseBSDF(Texture* _albedo) {
		albedo = _albedo;
	}
	
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf) {
		// Sample wi from Cosine Hemisphere Sampling (in z-up space)
		Vec3 wi = SamplingDistributions::cosineSampleHemisphere(sampler->next(), sampler->next());

		// PDF = cos(theta) / pi
		pdf = wi.z * M_1_PI;

		// BSDF = albedo / pi
		reflectedColour = albedo->sample(shadingData.tu, shadingData.tv) * M_1_PI;

		// Transform to world
		return shadingData.frame.toWorld(wi);
	}

	Colour evaluate(const ShadingData& shadingData, const Vec3& wi) {
		return albedo->sample(shadingData.tu, shadingData.tv) * M_1_PI;
	}

	float PDF(const ShadingData& shadingData, const Vec3& wi) {
		Vec3 wiLocal = shadingData.frame.toLocal(wi);
		return SamplingDistributions::cosineHemispherePDF(wiLocal);
	}

	bool isPureSpecular() {
		return false;
	}

	bool isTwoSided() {
		return true;
	}

	float mask(const ShadingData& shadingData) {
		return albedo->sampleAlpha(shadingData.tu, shadingData.tv);
	}
};

// Done 100% Sure
class MirrorBSDF : public BSDF {
public:
	Texture* albedo;

	MirrorBSDF() = default;
	MirrorBSDF(Texture* _albedo) {
		albedo = _albedo;
	}

	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf) {
		// Convert wo to local space
		Vec3 woLocal = shadingData.frame.toLocal(shadingData.wo);

		// Need reflectance in direction wr where w = wo
		// Reflect x and y
		Vec3 wr(-woLocal.x, -woLocal.y, woLocal.z);

		// BSDF = albedo / Dot(wr, n)
		// If wr is in local space, Dot(wr, n) = wr.z
		reflectedColour = albedo->sample(shadingData.tu, shadingData.tv) / wr.z;

		// PDF = 1 (Perfect/Specular Reflection)
		pdf = 1.f;

		// Convert back to world space
		return shadingData.frame.toWorld(wr);
	}

	Colour evaluate(const ShadingData& shadingData, const Vec3& wi) {
		Vec3 wiLocal = shadingData.frame.toLocal(wi);
		return albedo->sample(shadingData.tu, shadingData.tv) / wiLocal.z;
	}

	float PDF(const ShadingData& shadingData, const Vec3& wi) {
		return 0.f;
	}

	bool isPureSpecular() {
		return true;
	}

	bool isTwoSided() {
		return true;
	}

	float mask(const ShadingData& shadingData) {
		return albedo->sampleAlpha(shadingData.tu, shadingData.tv);
	}
};

// Nope...
class ConductorBSDF : public BSDF {
public:
	Texture* albedo;
	Colour eta;
	Colour k;
	float alpha;

	ConductorBSDF() = default;
	ConductorBSDF(Texture* _albedo, Colour _eta, Colour _k, float roughness) {
		albedo = _albedo;
		eta = _eta;
		k = _k;
		alpha = 1.62142f * sqrtf(roughness);
	}

	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf) {
		// Replace this with OrenNayar sampling code
		Vec3 wi = SamplingDistributions::cosineSampleHemisphere(sampler->next(), sampler->next());
		pdf = wi.z / M_PI;
		reflectedColour = albedo->sample(shadingData.tu, shadingData.tv) / M_PI;
		wi = shadingData.frame.toWorld(wi);
		return wi;
	}

	Colour evaluate(const ShadingData& shadingData, const Vec3& wi) {
		// Replace this with OrenNayar evaluation code
		return albedo->sample(shadingData.tu, shadingData.tv) / M_PI;
	}

	float PDF(const ShadingData& shadingData, const Vec3& wi) {
		// Replace this with OrenNayar PDF
		Vec3 wiLocal = shadingData.frame.toLocal(wi);
		return SamplingDistributions::cosineHemispherePDF(wiLocal);
	}

	bool isPureSpecular() {
		return false;
	}

	bool isTwoSided() {
		return true;
	}

	float mask(const ShadingData& shadingData) {
		return albedo->sampleAlpha(shadingData.tu, shadingData.tv);
	}
};

// Done 100% Sure
class GlassBSDF : public BSDF {
public:
	Texture* albedo;
	float intIOR;
	float extIOR;

	GlassBSDF() = default;
	GlassBSDF(Texture* _albedo, float _intIOR, float _extIOR) {
		albedo = _albedo;
		intIOR = _intIOR;
		extIOR = _extIOR;
	}

	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf) {
		Vec3 woLocal = shadingData.frame.toLocal(shadingData.wo);
		float cosTheta = woLocal.z;
		float IOR = (cosTheta > 0.f) ? (extIOR / intIOR) : (intIOR / extIOR);
		float fresnelConst = ShadingHelper::fresnelDielectric(cosTheta, intIOR, extIOR);

		if (sampler->next() < fresnelConst) {
			// Reflect
			pdf = fresnelConst;
			Vec3 wr(-woLocal.x, -woLocal.y, woLocal.z);
			reflectedColour = albedo->sample(shadingData.tu, shadingData.tv) * fresnelConst / wr.z;
			return shadingData.frame.toWorld(wr);
		} else {
			// Refract (Transmit)
			float sinTheta = std::max(sqrtf(1.f - powf(cosTheta, 2)), 0.f);
			if (IOR * sinTheta >= 1.f) {
				// Total Internal Reflection
				pdf = 1.f;
				Vec3 wr(-woLocal.x, -woLocal.y, woLocal.z);
				reflectedColour = albedo->sample(shadingData.tu, shadingData.tv) / wr.z;
				return shadingData.frame.toWorld(wr);
			} else {
				pdf = 1.f - fresnelConst;
				float wtZ = (cosTheta > 0) ? -sqrtf(1.f - powf(IOR, 2) * powf(sinTheta, 2)) : sqrtf(1.f - powf(IOR, 2) * powf(sinTheta, 2));
				Vec3 wt(-woLocal.x * IOR, -woLocal.y * IOR, wtZ);
				reflectedColour = albedo->sample(shadingData.tu, shadingData.tv) * (1.f - fresnelConst) * IOR * IOR / wt.z;
				return shadingData.frame.toWorld(wt);
			}
		}
	}

	Colour evaluate(const ShadingData& shadingData, const Vec3& wi) {
		Vec3 woLocal = shadingData.frame.toLocal(shadingData.wo);
		float cosTheta = woLocal.z;
		float IOR = (cosTheta > 0.f) ? (extIOR / intIOR) : (intIOR / extIOR);
		float reflectence = ShadingHelper::fresnelDielectric(cosTheta, intIOR, extIOR);
		float refractance = 1.f - reflectence;

		if (reflectence > refractance) {
			// Reflect
			Vec3 wr(-woLocal.x, -woLocal.y, woLocal.z);
			return albedo->sample(shadingData.tu, shadingData.tv) * reflectence / wr.z;
		} else {
			// Refract (Transmit)
			float sinTheta = std::max(sqrtf(1.f - powf(cosTheta, 2)), 0.f);
			if (IOR * sinTheta >= 1.f) {
				// Total Internal Reflection
				Vec3 wr(-woLocal.x, -woLocal.y, woLocal.z);
				return albedo->sample(shadingData.tu, shadingData.tv) / wr.z;
			} else {
				float wtZ = (cosTheta > 0) ? -sqrtf(1.f - powf(IOR, 2) * powf(sinTheta, 2)) : sqrtf(1.f - powf(IOR, 2) * powf(sinTheta, 2));
				Vec3 wt(-woLocal.x * IOR, -woLocal.y * IOR, wtZ);
				return albedo->sample(shadingData.tu, shadingData.tv) * refractance * IOR * IOR / wt.z;
			}
		}
	}

	float PDF(const ShadingData& shadingData, const Vec3& wi) {
		return 0.f;
	}

	bool isPureSpecular() {
		return true;
	}

	bool isTwoSided() {
		return false;
	}

	float mask(const ShadingData& shadingData) {
		return albedo->sampleAlpha(shadingData.tu, shadingData.tv);
	}
};

class DielectricBSDF : public BSDF {
public:
	Texture* albedo;
	float intIOR;
	float extIOR;
	float alpha;

	DielectricBSDF() = default;
	DielectricBSDF(Texture* _albedo, float _intIOR, float _extIOR, float roughness) {
		albedo = _albedo;
		intIOR = _intIOR;
		extIOR = _extIOR;
		alpha = 1.62142f * sqrtf(roughness);
	}

	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf) {
		// Replace this with Dielectric sampling code
		Vec3 wi = SamplingDistributions::cosineSampleHemisphere(sampler->next(), sampler->next());
		pdf = wi.z / M_PI;
		reflectedColour = albedo->sample(shadingData.tu, shadingData.tv) / M_PI;
		wi = shadingData.frame.toWorld(wi);
		return wi;
	}

	Colour evaluate(const ShadingData& shadingData, const Vec3& wi) {
		// Replace this with Dielectric evaluation code
		return albedo->sample(shadingData.tu, shadingData.tv) / M_PI;
	}

	float PDF(const ShadingData& shadingData, const Vec3& wi) {
		// Replace this with Dielectric PDF
		Vec3 wiLocal = shadingData.frame.toLocal(wi);
		return SamplingDistributions::cosineHemispherePDF(wiLocal);
	}

	bool isPureSpecular() {
		return false;
	}

	bool isTwoSided() {
		return false;
	}

	float mask(const ShadingData& shadingData) {
		return albedo->sampleAlpha(shadingData.tu, shadingData.tv);
	}
};

class OrenNayarBSDF : public BSDF {
public:
	Texture* albedo;
	float sigma;

	OrenNayarBSDF() = default;
	OrenNayarBSDF(Texture* _albedo, float _sigma) {
		albedo = _albedo;
		sigma = _sigma;
	}

	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf) {
		// Replace this with OrenNayar sampling code
		Vec3 wi = SamplingDistributions::cosineSampleHemisphere(sampler->next(), sampler->next());
		pdf = wi.z / M_PI;
		reflectedColour = albedo->sample(shadingData.tu, shadingData.tv) / M_PI;
		wi = shadingData.frame.toWorld(wi);
		return wi;
	}

	Colour evaluate(const ShadingData& shadingData, const Vec3& wi) {
		// Replace this with OrenNayar evaluation code
		return albedo->sample(shadingData.tu, shadingData.tv) / M_PI;
	}

	float PDF(const ShadingData& shadingData, const Vec3& wi) {
		// Replace this with OrenNayar PDF
		Vec3 wiLocal = shadingData.frame.toLocal(wi);
		return SamplingDistributions::cosineHemispherePDF(wiLocal);
	}

	bool isPureSpecular() {
		return false;
	}

	bool isTwoSided() {
		return true;
	}

	float mask(const ShadingData& shadingData) {
		return albedo->sampleAlpha(shadingData.tu, shadingData.tv);
	}
};

// Using Phong Model for PlasticBSDF, extent to Blinn or LaFortune if time left
class PlasticBSDF : public BSDF {
public:
	Texture* albedo;
	float intIOR;
	float extIOR;
	float alpha;

	PlasticBSDF() = default;
	PlasticBSDF(Texture* _albedo, float _intIOR, float _extIOR, float roughness) {
		albedo = _albedo;
		intIOR = _intIOR;
		extIOR = _extIOR;
		alpha = 1.62142f * sqrtf(roughness);
	}

	float alphaToPhongExponent() {
		return (2.0f / SQ(std::max(alpha, 0.001f))) - 2.0f;
	}

	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf) {
		// Replace this with Dielectric sampling code
		Vec3 wi = SamplingDistributions::cosineSampleHemisphere(sampler->next(), sampler->next());
		pdf = wi.z / M_PI;
		reflectedColour = albedo->sample(shadingData.tu, shadingData.tv) / M_PI;
		wi = shadingData.frame.toWorld(wi);
		return wi;
	}

	Colour evaluate(const ShadingData& shadingData, const Vec3& wi) {
		// Replace this with Dielectric evaluation code
		return albedo->sample(shadingData.tu, shadingData.tv) / M_PI;
	}

	float PDF(const ShadingData& shadingData, const Vec3& wi) {
		// Replace this with Dielectric PDF
		Vec3 wiLocal = shadingData.frame.toLocal(wi);
		return SamplingDistributions::cosineHemispherePDF(wiLocal);
	}

	bool isPureSpecular() {
		return false;
	}

	bool isTwoSided() {
		return true;
	}

	float mask(const ShadingData& shadingData) {
		return albedo->sampleAlpha(shadingData.tu, shadingData.tv);
	}
};

// None of the provided scenes seemed to have LayeredBSDF as a material type when I checked the .json files
// For that reason, LayeredBSDF will not be implemented in this assignment, unless there's extra time left
// However, for scenes like Fur Ball (Bitterli), or Curly Hair/Straight Hair (Yuksel), LayeredBSDF's a must
class LayeredBSDF : public BSDF {
public:
	BSDF* base;
	Colour sigmaa;
	float thickness;
	float intIOR;
	float extIOR;

	LayeredBSDF() = default;
	LayeredBSDF(BSDF* _base, Colour _sigmaa, float _thickness, float _intIOR, float _extIOR) {
		base = _base;
		sigmaa = _sigmaa;
		thickness = _thickness;
		intIOR = _intIOR;
		extIOR = _extIOR;
	}

	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf) {
		// Add code to include layered sampling
		return base->sample(shadingData, sampler, reflectedColour, pdf);
	}

	Colour evaluate(const ShadingData& shadingData, const Vec3& wi) {
		// Add code for evaluation of layer
		return base->evaluate(shadingData, wi);
	}

	float PDF(const ShadingData& shadingData, const Vec3& wi) {
		// Add code to include PDF for sampling layered BSDF
		return base->PDF(shadingData, wi);
	}

	bool isPureSpecular() {
		return base->isPureSpecular();
	}

	bool isTwoSided() {
		return true;
	}

	float mask(const ShadingData& shadingData) {
		return base->mask(shadingData);
	}
};