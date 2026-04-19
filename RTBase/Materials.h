#pragma once

#include "Core.h"
#include "Imaging.h"
#include "Sampling.h"

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
		// Add code here
		cosTheta = std::max(std::min(cosTheta, 1.f), -1.f);

		// Find eta (depends on direction/shadingData.wo's sign)
		float eta = (cosTheta < 0.f) ? (iorInt / iorExt) : (iorExt / iorInt);
		
		// Given cosTheta_i, calculate cosTheta_t from Snell's Law
		float sinTheta_i = 1.f - powf(cosTheta, 2);
		if (powf(eta, 2) * powf(sinTheta_i, 2) > 1.f) return 1.f;  // What if eta^2 sin^2(theta_i) > 1 ?
		float cosTheta_t = sqrtf(1.f - (powf(eta, 2) * powf(sinTheta_i, 2)));
		
		// Return the squared average of perpendicular and parallel
		float fParallel = (cosTheta - eta * cosTheta_t) / (cosTheta + eta * cosTheta_t);
		float fPerpendicular = (eta * cosTheta - cosTheta_t) / (eta * cosTheta + cosTheta_t);
		return (powf(fParallel, 2) + powf(fPerpendicular, 2)) * 0.5f;
	}

	static Colour fresnelConductor(float cosTheta, Colour ior, Colour k) {
		// Add code here
		// Given cosTheta_i, find sinTheta_i
		float sinTheta = 1.f - powf(cosTheta, 2);
		Colour cosTheta_i(cosTheta, cosTheta, cosTheta);
		Colour sinTheta_i(sinTheta, sinTheta, sinTheta);

		// Return the squared average of perpendicular and parallel
		Colour fParallel = (
			(((ior * ior) + (k * k)) * (cosTheta_i * cosTheta_i)) - (ior * 2 * cosTheta_i) + (sinTheta_i) /
			(((ior * ior) + (k * k)) * (cosTheta_i * cosTheta_i)) + (ior * 2 * cosTheta_i) + (sinTheta_i)
		);
		Colour fPerpendicular = (
			(ior * ior) + (k * k) - (ior * 2 * cosTheta_i) + (cosTheta_i * cosTheta_i) /
			(ior * ior) + (k * k) + (ior * 2 * cosTheta_i) + (cosTheta_i * cosTheta_i)
		);
		return ((fParallel * fParallel) + (fPerpendicular * fPerpendicular)) / 2.f;
	}

	static float lambdaGGX(Vec3 wi, float alpha) {
		// Add code here
		return 1.0f;
	}

	static float Gggx(Vec3 wi, Vec3 wo, float alpha) {
		// Add code here
		return 1.0f;
	}
	
	static float Dggx(Vec3 h, float alpha) {
		// Add code here
		return 1.0f;
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
		// Add correct sampling code here
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
		// Add correct PDF code here
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
		// Replace this with Mirror sampling code
		// Convert shadingData.wo to local space
		Vec3 woLocal = shadingData.frame.toLocal(shadingData.wo);

		// Reflect local x and y
		woLocal.x = -woLocal.x;
		woLocal.y = -woLocal.y;

		// Convert back to world space
		Vec3 woWorld = shadingData.frame.toWorld(woLocal);
		reflectedColour = evaluate(shadingData, woWorld);
		pdf = 1.f;
		return woWorld;
	}

	Colour evaluate(const ShadingData& shadingData, const Vec3& wi) {
		// Replace this with Mirror evaluation code
		return albedo->sample(shadingData.tu, shadingData.tv) / Dot(wi, shadingData.sNormal);
	}

	float PDF(const ShadingData& shadingData, const Vec3& wi) {
		// Replace this with Mirror PDF
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
		// Replace this with Conductor sampling code
		Vec3 wi = SamplingDistributions::cosineSampleHemisphere(sampler->next(), sampler->next());
		pdf = wi.z / M_PI;
		reflectedColour = albedo->sample(shadingData.tu, shadingData.tv) / M_PI;
		wi = shadingData.frame.toWorld(wi);
		return wi;
	}

	Colour evaluate(const ShadingData& shadingData, const Vec3& wi) {
		// Replace this with Conductor evaluation code
		return albedo->sample(shadingData.tu, shadingData.tv) / M_PI;
	}

	float PDF(const ShadingData& shadingData, const Vec3& wi) {
		// Replace this with Conductor PDF
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
		// Replace this with Glass sampling code
		Vec3 wi = SamplingDistributions::cosineSampleHemisphere(sampler->next(), sampler->next());
		pdf = wi.z / M_PI;
		reflectedColour = albedo->sample(shadingData.tu, shadingData.tv) / M_PI;
		wi = shadingData.frame.toWorld(wi);
		return wi;
	}

	Colour evaluate(const ShadingData& shadingData, const Vec3& wi) {
		// Replace this with Glass evaluation code
		return albedo->sample(shadingData.tu, shadingData.tv) / M_PI;
	}

	float PDF(const ShadingData& shadingData, const Vec3& wi) {
		// Replace this with GlassPDF
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