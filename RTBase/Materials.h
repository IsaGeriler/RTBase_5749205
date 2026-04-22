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
	static float fresnelDielectric(float cosTheta, float iorInt, float iorExt) {
		// Find eta (depends on direction/shadingData.wo's sign)
		float cosTheta_i = std::max(std::min(cosTheta, 1.f), -1.f);
		float eta = (cosTheta_i > 0.f) ? (iorInt / iorExt) : (iorExt / iorInt);
		if (cosTheta_i <= 0.f) cosTheta_i = fabs(cosTheta_i);

		// Given cosTheta_i, calculate cosTheta_t from Snell's Law
		float sinTheta_i = sqrtf(std::max(1.f - powf(cosTheta_i, 2), 0.f));
		float sinTheta_t = eta * sinTheta_i;

		// What if sinTheta_t is greater or equal than one
		// Total Internal Reflection
		if (sinTheta_t >= 1.f) return 1.f;
		float cosTheta_t = sqrtf(std::max(1.f - powf(sinTheta_t, 2), 0.f));
		
		// Return the squared average of perpendicular and parallel
		float fParallel = (cosTheta_i - eta * cosTheta_t) / (cosTheta_i + eta * cosTheta_t);
		float fPerpendicular = (eta * cosTheta_i - cosTheta_t) / (eta * cosTheta_i + cosTheta_t);
		return (powf(fParallel, 2) + powf(fPerpendicular, 2)) * 0.5f;
	}

	static Colour fresnelConductor(float cosTheta, Colour ior, Colour k) {
		float sinTheta = sqrtf(1.f - powf(fabs(cosTheta), 2));
		Colour cosTheta_i(fabs(cosTheta), fabs(cosTheta), fabs(cosTheta));
		Colour sinTheta_i(sinTheta, sinTheta, sinTheta);

		// Return the squared average of perpendicular and parallel
		Colour fParallelSquared = (
			(((ior * ior) + (k * k)) * (cosTheta_i * cosTheta_i)) - (ior * 2 * cosTheta_i) + (sinTheta_i) /
			(((ior * ior) + (k * k)) * (cosTheta_i * cosTheta_i)) + (ior * 2 * cosTheta_i) + (sinTheta_i)
		);
		Colour fPerpendicularSquared = (
			(ior * ior) + (k * k) - (ior * 2 * cosTheta_i) + (cosTheta_i * cosTheta_i) /
			(ior * ior) + (k * k) + (ior * 2 * cosTheta_i) + (cosTheta_i * cosTheta_i)
		);
		return (fParallelSquared + fPerpendicularSquared) * 0.5f;
	}

	static float lambdaGGX(Vec3 wi, float alpha) {
		// Isotropic Lambda Function for GGX (Trowbridge-Reitz)
		float cosTheta = wi.z;
		float sinTheta = sqrtf(std::max(1.f - powf(cosTheta, 2), 0.f));
		float tanTheta = std::fabs(sinTheta / cosTheta);
		return (sqrtf(1 + powf(alpha, 2) * powf(tanTheta, 2)) - 1.f) * 0.5f;
	}

	static float Gggx(Vec3 wi, Vec3 wo, float alpha) {
		// Assume masking and shadowing are statistically independent
		// G(wo,wi) = G1(wo,wm) G1(wi,wm)
		// For GGX (Trowbridge-Reitz) = 1.f / (1.f + Λ(wo) + Λ(wi))
		return 1.f / (1.f + lambdaGGX(wo, alpha) + lambdaGGX(wi, alpha));
	}
	
	static float Dggx(Vec3 h, float alpha) {
		// Isotropic Distribution for GGX (Trowbridge-Reitz)
		float cosTheta = h.z;
		return powf(alpha, 2) / (M_PI * powf(powf(cosTheta, 2) * (powf(alpha, 2) - 1.f) + 1.f, 2));
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

class DiffuseBSDF : public BSDF {
public:
	Texture* albedo;

	DiffuseBSDF() = default;
	DiffuseBSDF(Texture* _albedo) {
		albedo = _albedo;
	}
	
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf) {
		Vec3 wi = SamplingDistributions::cosineSampleHemisphere(sampler->next(), sampler->next());
		pdf = wi.z / M_PI;
		reflectedColour = albedo->sample(shadingData.tu, shadingData.tv) / M_PI;
		wi = shadingData.frame.toWorld(wi);
		return wi;
	}

	Colour evaluate(const ShadingData& shadingData, const Vec3& wi) {
		return albedo->sample(shadingData.tu, shadingData.tv) / M_PI;
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

class MirrorBSDF : public BSDF {
public:
	Texture* albedo;

	MirrorBSDF() = default;
	MirrorBSDF(Texture* _albedo) {
		albedo = _albedo;
	}

	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf) {
		// Convert shadingData.wo to local space
		Vec3 wo = shadingData.frame.toLocal(shadingData.wo);
		
		// Reflect local x and y
		Vec3 wr(-wo.x, -wo.y, wo.z);

		// Convert back to world space
		pdf = 1.f;
		reflectedColour = albedo->sample(shadingData.tu, shadingData.tv) / Dot(wr, shadingData.sNormal);
		return shadingData.frame.toWorld(wr);
	}

	Colour evaluate(const ShadingData& shadingData, const Vec3& wi) {
		return albedo->sample(shadingData.tu, shadingData.tv) / Dot(wi, shadingData.sNormal);
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
		// Obtain wo and convert to local
		Vec3 wo = shadingData.frame.toLocal(shadingData.wo);

		// Can only sample visible normal from wo
		if (wo.z <= 0.f) {
			pdf = 0.f;
			reflectedColour = Colour(0.f, 0.f, 0.f);
			return Vec3(0.f, 0.f, 0.f);
		}

		if (alpha < EPSILON) {
			// Treat as a mirror with Conductor Fresnel
			pdf = 1.f;
			Vec3 wr(-wo.x, -wo.y, wo.z);
			Colour Fwr = ShadingHelper::fresnelConductor(wr.z, eta, k);
			reflectedColour = albedo->sample(shadingData.tu, shadingData.tv) * Fwr / wr.z * pdf;
			return shadingData.frame.toWorld(wr);
		}

		// Sample theta and phi from CDF inversion
		float r1 = sampler->next();
		float r2 = sampler->next();

		float theta_m = acosf(sqrtf((1.f - r1) / (r1 * (powf(alpha, 2) - 1.f) + 1.f)));
		float phi_m = 2.f * M_PI * r2;

		// Sampled wm from spherical coordinates
		Vec3 wm(sinf(theta_m) * cosf(phi_m), sinf(theta_m) * sinf(phi_m), cos(theta_m));

		// Obtain light reflected across microfacet
		Vec3 wi = -wo + (wm * 2 * Dot(wm, wo));
		
		// Edge Case Handling
		float cosTheta_o = wo.z;
		float cosTheta_i = wi.z;

		if (cosTheta_o == 0.f || cosTheta_i == 0.f) {
			pdf = 0.f;
			reflectedColour = Colour(0.f, 0.f, 0.f);
			return Vec3(0.f, 0.f, 0.f);
		}

		if (wm.x == 0.f && wm.y == 0.f && wm.z == 0.f) {
			pdf = 0.f;
			reflectedColour = Colour(0.f, 0.f, 0.f);
			return Vec3(0.f, 0.f, 0.f);
		}

		// Cook-Torrance BRDF
		float G = ShadingHelper::Gggx(wi, wo, alpha);
		float D = ShadingHelper::Dggx(wm, alpha);
		Colour F = ShadingHelper::fresnelConductor(Dot(wi, wm), eta, k);
		Colour BRDF = (F * G * D) / (4 * cosTheta_o * cosTheta_i);
		pdf = (D * wm.z) / (4.f * Dot(wo, wm));
		reflectedColour = albedo->sample(shadingData.tu, shadingData.tv) * BRDF / pdf;
		return shadingData.frame.toWorld(wi);
	}

	Colour evaluate(const ShadingData& shadingData, const Vec3& wi) {
		// Replace this with Conductor evaluation code
		Vec3 wiLocal = shadingData.frame.toLocal(wi);
		Vec3 woLocal = shadingData.frame.toLocal(shadingData.wo);

		// Can only sample visible normal from wo
		if (woLocal.z <= 0.f) return Colour(0.f, 0.f, 0.f);

		// Treat as a mirror with Conductor Fresnel
		if (alpha < EPSILON) return Colour(0.f, 0.f, 0.f);

		// Edge Case
		Vec3 wm = (wiLocal + woLocal).normalize();
		if (wm.x == 0.f && wm.y == 0.f && wm.z == 0.f) return Colour(0.f, 0.f, 0.f);

		float G = ShadingHelper::Gggx(wiLocal, woLocal, alpha);
		float D = ShadingHelper::Dggx(wm, alpha);
		Colour F = ShadingHelper::fresnelConductor(Dot(woLocal, wm), eta, k);
		Colour BRDF = (F * G * D) / (4.f * woLocal.z * wiLocal.z);
		return albedo->sample(shadingData.tu, shadingData.tv) * BRDF;
	}

	float PDF(const ShadingData& shadingData, const Vec3& wi) {
		Vec3 wiLocal = shadingData.frame.toLocal(wi);
		Vec3 woLocal = shadingData.frame.toLocal(shadingData.wo);

		// Can only sample visible normal from wo
		if (woLocal.z <= 0.f) return 0.f;

		// Treat as a mirror with Conductor Fresnel
		if (alpha < EPSILON) return 0.f;

		// Edge Case
		Vec3 wm = (wiLocal + woLocal).normalize();
		if (wm.x == 0.f && wm.y == 0.f && wm.z == 0.f) return 0.f;

		float D = ShadingHelper::Dggx(wm, alpha);
		return (D * wm.z) / (4.f * Dot(woLocal, wm));
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

class GlassBSDF : public BSDF
{
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
		float IOR = (cosTheta > 0.f) ? (intIOR / extIOR) : (extIOR / intIOR);
		float fresnelConst = ShadingHelper::fresnelDielectric(cosTheta, intIOR, extIOR);

		if (sampler->next() < fresnelConst) {
			// Reflect
			pdf = fresnelConst;
			Vec3 wr(-woLocal.x, -woLocal.y, woLocal.z);
			reflectedColour = albedo->sample(shadingData.tu, shadingData.tv) * fresnelConst / Dot(wr, shadingData.sNormal);
			return shadingData.frame.toWorld(wr);
		} else {
			// Refract (Transmit)
			float sinTheta = std::max(sqrtf(1.f - powf(cosTheta, 2)), 0.f);
			if (powf(IOR * sinTheta, 2) > 1.f) {
				// Total Internal Reflection
				pdf = 1.f;
				Vec3 wr(-woLocal.x, -woLocal.y, woLocal.z);
				reflectedColour = albedo->sample(shadingData.tu, shadingData.tv) / Dot(wr, shadingData.sNormal);
				return shadingData.frame.toWorld(wr);
			} else {
				pdf = 1.f - fresnelConst;
				float wtZ = cosTheta > 0 ? -sqrtf(1.f - powf(sinTheta, 2)) : sqrtf(1.f - powf(sinTheta, 2));
				Vec3 wt(-woLocal.x * IOR, -woLocal.y * IOR, wtZ);
				reflectedColour = albedo->sample(shadingData.tu, shadingData.tv) * (1.f - fresnelConst) * IOR * IOR / Dot(wt, shadingData.sNormal);
				return shadingData.frame.toWorld(wt);
			}
		}
	}

	Colour evaluate(const ShadingData& shadingData, const Vec3& wi) {
		Vec3 wiLocal = shadingData.frame.toLocal(wi);
		Vec3 woLocal = shadingData.frame.toLocal(shadingData.wo);

		float cosTheta = woLocal.z;
		float IOR = (cosTheta > 0.f) ? (intIOR / extIOR) : (extIOR / intIOR);
		float reflectence = ShadingHelper::fresnelDielectric(cosTheta, intIOR, extIOR);
		float refractance = 1.f - reflectence;

		if (reflectence > refractance) {
			// Reflect
			Vec3 wr(-woLocal.x, -woLocal.y, woLocal.z);
			return albedo->sample(shadingData.tu, shadingData.tv) * reflectence / Dot(wr, shadingData.sNormal);
		} else {
			// Refract (Transmit)
			float sinTheta = std::max(sqrtf(1.f - powf(cosTheta, 2)), 0.f);
			if (powf(IOR * sinTheta, 2) > 1.f) {
				// Total Internal Reflection
				Vec3 wr(-woLocal.x, -woLocal.y, woLocal.z);
				return albedo->sample(shadingData.tu, shadingData.tv) / Dot(wr, shadingData.sNormal);
			} else {
				float wtZ = cosTheta > 0 ? -sqrtf(1.f - powf(sinTheta, 2)) : sqrtf(1.f - powf(sinTheta, 2));
				Vec3 wt(-woLocal.x * IOR, -woLocal.y * IOR, wtZ);
				return albedo->sample(shadingData.tu, shadingData.tv) * refractance * IOR * IOR / Dot(wt, shadingData.sNormal);
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

// Using Phong Model for PlasticBSDF
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
		// Replace this with Plastic sampling code
		Vec3 wi = SamplingDistributions::cosineSampleHemisphere(sampler->next(), sampler->next());
		pdf = wi.z / M_PI;
		reflectedColour = albedo->sample(shadingData.tu, shadingData.tv) / M_PI;
		wi = shadingData.frame.toWorld(wi);
		return wi;

		// Sample a lobe
		//float thetaLobe = acosf(powf(sampler->next(), 1.f / (alphaToPhongExponent() + 1)));
		//float phiLobe = 2.f * M_PI * sampler->next();
		//Vec3 wLobe(sinf(thetaLobe) * cosf(phiLobe), sinf(thetaLobe) * sinf(phiLobe), cosf(thetaLobe));

		// Create frame aligned along

		// Rotate
	}

	Colour evaluate(const ShadingData& shadingData, const Vec3& wi) {
		// Replace this with Plastic evaluation code
		return albedo->sample(shadingData.tu, shadingData.tv) / M_PI;
	}

	float PDF(const ShadingData& shadingData, const Vec3& wi) {
		// Replace this with Plastic PDF
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