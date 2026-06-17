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
		// Find eta, depends on shadingData.wo's sign
		float cosThetaI = std::max(std::min(cosTheta, 1.f), -1.f);
		float eta = iorExt / iorInt;
		
		if (cosThetaI < 0.f) {
			eta = 1.f / eta;
			cosThetaI = fabs(cosThetaI);
		}

		// Given cosThetaI, calculate cosThetaT from Snell's Law
		float sinThetaI = sqrtf(std::max(1.f - powf(cosThetaI, 2), 0.f));
		float sin2ThetaT = powf(eta * sinThetaI, 2);

		// Handle Total Internal Reflection
		if (sin2ThetaT >= 1.f) return 1.f;
		float cosThetaT = sqrtf(std::max(1.f - sin2ThetaT, 0.f));
		
		// Return the squared average of perpendicular and parallel
		float fParallel = (cosThetaI - eta * cosThetaT) / (cosThetaI + eta * cosThetaT);
		float fPerpendicular = (eta * cosThetaI - cosThetaT) / (eta * cosThetaI + cosThetaT);
		return (powf(fParallel, 2) + powf(fPerpendicular, 2)) * 0.5f;
	}

	// Done 100% Sure
	static Colour fresnelConductor(float cosTheta, Colour ior, Colour k) {
		float fCosThetaI = std::max(std::min(cosTheta, 1.f), -1.f);
		float fSinThetaI = sqrtf(std::max(1.f - powf(fCosThetaI, 2), 0.f));
		Colour cosThetaI(fCosThetaI, fCosThetaI, fCosThetaI);
		Colour sinThetaI(fSinThetaI, fSinThetaI, fSinThetaI);

		// Return the squared average of perpendicular and parallel
		Colour parallelSquared = (
			(((ior * ior + k * k) * (cosThetaI * cosThetaI)) - (ior * 2.f * cosThetaI) + (sinThetaI * sinThetaI)) /
			(((ior * ior + k * k) * (cosThetaI * cosThetaI)) + (ior * 2.f * cosThetaI) + (sinThetaI * sinThetaI))
		);
		Colour perpendicularSquared = (
			(ior * ior + k * k - (ior * 2.f * cosThetaI) + (cosThetaI * cosThetaI)) /
			(ior * ior + k * k + (ior * 2.f * cosThetaI) + (cosThetaI * cosThetaI))
		);
		return (parallelSquared + perpendicularSquared) * 0.5f;
	}

	// Done 100% Sure
	static float lambdaGGX(Vec3 wi, float alpha) {
		// Isotropic Lambda Function for GGX (Trowbridge-Reitz)
		// Avoid division by zero
		if (wi.z <= 0.f) return 0.f;
		float cosTheta = wi.z;
		float sinTheta = sqrtf(std::max(1.f - powf(cosTheta, 2), 0.f));
		float tanTheta = std::fabs(sinTheta / cosTheta);
		return (sqrtf(1.f + powf(alpha, 2) * powf(tanTheta, 2)) - 1.f) * 0.5f;
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
		// Avoid division by zero
		if (h.z <= 0.f) return 0.f;
		float denom = M_PI * powf(powf(h.z, 2) * (powf(alpha, 2) - 1.f) + 1.f, 2);
		if (denom <= 0.f) return 0.f;
		return powf(alpha, 2) / denom;
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
		
		// Transform to world after sampling
		wi = shadingData.frame.toWorld(wi);
		
		pdf = PDF(shadingData, wi);					  // PDF = cos(theta) / pi
		reflectedColour = evaluate(shadingData, wi);  // BSDF = albedo / pi
		return wi;
	}

	Colour evaluate(const ShadingData& shadingData, const Vec3& wi) {
		Vec3 wiLocal = shadingData.frame.toLocal(wi);
		if (wiLocal.z <= 0.f) return Colour(0.f, 0.f, 0.f);
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
		if (wr.z <= 0.f) reflectedColour = Colour(0.f, 0.f, 0.f);
		else reflectedColour = albedo->sample(shadingData.tu, shadingData.tv) / wr.z;
		
		// PDF = 1 (Perfect/Specular Reflection)
		pdf = 1.f;

		// Convert back to world space
		return shadingData.frame.toWorld(wr);
	}

	Colour evaluate(const ShadingData& shadingData, const Vec3& wi) {
		return Colour(0.f, 0.f, 0.f);
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

// Done 100% Sure
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
		// Convert wo to local space
		Vec3 wo = shadingData.frame.toLocal(shadingData.wo);

		// Can sample only visible normal from wo
		if (wo.z <= 0.f) {
			pdf = 0.f;
			reflectedColour = Colour(0.f, 0.f, 0.f);
			return Vec3(0.f, 0.f, 0.f);
		}

		// If alpha < epsilon, treat as a mirror with Conductor Fresnel
		if (alpha < EPSILON) {
			Vec3 wi(-wo.x, -wo.y, wo.z);
			if (wi.z <= 0) {
				pdf = 0.f;
				reflectedColour = Colour(0, 0, 0);
				return Vec3(0, 0, 0);
			}
			pdf = 1.f;
			Colour F = ShadingHelper::fresnelConductor(wi.z, eta, k);
			reflectedColour = albedo->sample(shadingData.tu, shadingData.tv) * F / wi.z;
			return shadingData.frame.toWorld(wi);
		}

		// Sampling an isotropic GGX
		float r1 = sampler->next();
		float r2 = sampler->next();

		float thetaM = acosf(sqrtf(((1.f - r1) / (r1 * (powf(alpha, 2) - 1.f) + 1.f))));
		float phiM = 2.f * M_PI * r2;
		Vec3 wm = SphericalCoordinates::sphericalToWorld(thetaM, phiM);
		
		// Light reflected across microfacet
		Vec3 wi = -wo + (wm * 2.f * Dot(wm, wo));
		wi = shadingData.frame.toWorld(wi);

		// Evaluate PDF and BSDF
		pdf = PDF(shadingData, wi);
		reflectedColour = evaluate(shadingData, wi);
		return wi;
	}

	Colour evaluate(const ShadingData& shadingData, const Vec3& wi) {
		// Convert to local space
		Vec3 woLocal = shadingData.frame.toLocal(shadingData.wo);
		Vec3 wiLocal = shadingData.frame.toLocal(wi);

		// Can sample only visible normal from wo
		if (woLocal.z <= 0.f || wiLocal.z <= 0.f) return Colour(0.f, 0.f, 0.f);
		
		// If alpha < epsilon, treat as a mirror with Conductor Fresnel
		if (alpha < EPSILON) return Colour(0.f, 0.f, 0.f);

		Vec3 wm = (wiLocal + woLocal).normalize();
		if (Dot(woLocal, wm) <= 0.f || Dot(wiLocal, wm) <= 0.f) return Colour(0.f, 0.f, 0.f);

		// Cook-Torrance BRDF
		// Masking-Shadowing
		float G = ShadingHelper::Gggx(wiLocal, woLocal, alpha);

		// Normal Distribution Function
		float D = ShadingHelper::Dggx(wm, alpha);

		// Fresnel
		Colour F = ShadingHelper::fresnelConductor(Dot(woLocal, wm), eta, k);

		// BRDF
		float cosThetaO = fabs(woLocal.z);
		float cosThetaI = fabs(wiLocal.z);
		if (cosThetaO * cosThetaI == 0.f) return Colour(0.f, 0.f, 0.f);
		return albedo->sample(shadingData.tu, shadingData.tv) * (F * G * D) / (4.f * cosThetaO * cosThetaI);
	}

	float PDF(const ShadingData& shadingData, const Vec3& wi) {
		// Convert to local space
		Vec3 wiLocal = shadingData.frame.toLocal(wi);
		Vec3 woLocal = shadingData.frame.toLocal(shadingData.wo);

		// Can only sample visible normal from wo
		if (woLocal.z <= 0.f || wiLocal.z <= 0.f) return 0.f;

		// Treat as a mirror with Conductor Fresnel
		if (alpha < EPSILON) return 0.f;
		
		Vec3 wm = (wiLocal + woLocal).normalize();
		if (Dot(woLocal, wm) <= 0.f || Dot(wiLocal, wm) <= 0.f) return 0.f;

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
		
		if (woLocal.z == 0.f) {
			pdf = 0.f;
			reflectedColour = Colour(0.f, 0.f, 0.f);
			return Vec3(0.f, 0.f, 0.f);
		}

		float cosThetaI = woLocal.z;
		float fresnel = ShadingHelper::fresnelDielectric(cosThetaI, intIOR, extIOR);
		float IOR = (cosThetaI > 0.f) ? (extIOR / intIOR) : (intIOR / extIOR);
		if (cosThetaI < 0.f) cosThetaI = fabs(cosThetaI);

		if (sampler->next() < fresnel) {
			// Reflection
			pdf = fresnel;
			Vec3 wr(-woLocal.x, -woLocal.y, woLocal.z);
			reflectedColour = albedo->sample(shadingData.tu, shadingData.tv) * fresnel / wr.z;
			return shadingData.frame.toWorld(wr);
		} else {
			// Refraction (Transmission)
			float sinThetaI = std::max(sqrtf(1.f - powf(cosThetaI, 2)), 0.f);
			float sin2ThetaT = powf(IOR * sinThetaI, 2);
			if (sin2ThetaT >= 1.f) {
				// Total Internal Reflection
				pdf = 1.f;
				Vec3 wr(-woLocal.x, -woLocal.y, woLocal.z);
				reflectedColour = albedo->sample(shadingData.tu, shadingData.tv) / wr.z;
				return shadingData.frame.toWorld(wr);
			} else {
				pdf = 1.f - fresnel;
				float cosThetaT = sqrtf(1.f - sin2ThetaT);
				float wtZ = (woLocal.z > 0.f) ? -cosThetaT : cosThetaT;
				Vec3 wt(-woLocal.x * IOR, -woLocal.y * IOR, wtZ);
				reflectedColour = albedo->sample(shadingData.tu, shadingData.tv) * (1.f - fresnel) * powf(IOR, 2) / wt.z;
				return shadingData.frame.toWorld(wt);
			}
		}
	}

	Colour evaluate(const ShadingData& shadingData, const Vec3& wi) {
		return Colour(0.f, 0.f, 0.f);
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

// I haven't seen much scenes with DielectricBSDF, so I am skipping this implementation due limited time
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
		// Sample wi from Cosine Hemisphere Sampling (in z-up space)
		Vec3 wi = SamplingDistributions::cosineSampleHemisphere(sampler->next(), sampler->next());

		// Transform to world after sampling
		wi = shadingData.frame.toWorld(wi);

		pdf = PDF(shadingData, wi);
		reflectedColour = evaluate(shadingData, wi);
		return wi;
	}

	Colour evaluate(const ShadingData& shadingData, const Vec3& wi) {
		// Replace this with Dielectric evaluation code
		Vec3 wiLocal = shadingData.frame.toLocal(wi);
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

// Done 100% Sure
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
		// Sample wi from Cosine Hemisphere Sampling (in z-up space)
		// OrenNayarBSDF is similar to the DiffuseBSDF
		Vec3 wi = SamplingDistributions::cosineSampleHemisphere(sampler->next(), sampler->next());

		// Transform to world after sampling
		wi = shadingData.frame.toWorld(wi);

		pdf = PDF(shadingData, wi);
		reflectedColour = evaluate(shadingData, wi);
		return wi;
	}

	Colour evaluate(const ShadingData& shadingData, const Vec3& wi) {
		// Convert wo to local space
		Vec3 woLocal = shadingData.frame.toLocal(shadingData.wo);
		Vec3 wiLocal = shadingData.frame.toLocal(wi);

		if (woLocal.z <= 0.f || wiLocal.z <= 0.f) return Colour(0.f, 0.f, 0.f);

		// Oren-Nayar Constants
		float A = 1.f - (powf(sigma, 2) / (2.f * (powf(sigma, 2) + 0.33f)));
		float B = (0.45f * powf(sigma, 2)) / (powf(sigma, 2) + 0.09f);

		// Oren-Nayar Approximation
		float thetaI = SphericalCoordinates::sphericalTheta(wiLocal);
		float thetaO = SphericalCoordinates::sphericalTheta(woLocal);

		float phiI = SphericalCoordinates::sphericalPhi(wiLocal);
		float phiO = SphericalCoordinates::sphericalPhi(woLocal);

		float OrenNayar = A + B * std::max(0.f, cosf(phiI - phiO) * sinf(std::max(thetaI, thetaO))) * tanf(std::min(thetaI, thetaO));
		Colour Diffuse = albedo->sample(shadingData.tu, shadingData.tv) * M_1_PI;
		return Diffuse * OrenNayar;
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
// PhongBSDF is implemented, although Blinn or LaFortune would be a better option
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
		Vec3 wo = shadingData.frame.toLocal(shadingData.wo);
		if (wo.z <= 0.f) {
			pdf = 0.f;
			reflectedColour = Colour(0.f, 0.f, 0.f);
			return Vec3(0.f, 0.f, 0.f);
		}
		
		// ks and kd
		float ks = ShadingHelper::fresnelDielectric(wo.z, intIOR, extIOR);
		float kd = 1.f - ks;

		// Random Variables
		float r1 = sampler->next();
		float r2 = sampler->next();

		// Glossy Part of Material
		Vec3 wi;
		if (sampler->next() < ks) {
			// Sample a lobe
			float e = alphaToPhongExponent();
			float thetaLobe = acosf(powf(r1, 1.f / (e + 1)));
			float phiLobe = 2.f * M_PI * r2;

			Vec3 wr(-wo.x, -wo.y, wo.z);
			Vec3 wLobe = SphericalCoordinates::sphericalToWorld(thetaLobe, phiLobe);

			// Create a frame along wr
			Frame Rwr;
			Rwr.Frame::fromVector(wr);

			// Rotate vector along the frame
			wi = Rwr.toWorld(wLobe);
		}
		// Diffuse Part of Material
		else wi = SamplingDistributions::cosineSampleHemisphere(r1, r2);

		// PDF and BSDF
		wi = shadingData.frame.toWorld(wi);
		pdf = PDF(shadingData, wi);
		reflectedColour = evaluate(shadingData, wi);
		return wi;
	}

	Colour evaluate(const ShadingData& shadingData, const Vec3& wi) {
		Vec3 wiLocal = shadingData.frame.toLocal(wi);
		Vec3 woLocal = shadingData.frame.toLocal(shadingData.wo);

		if (wiLocal.z <= 0.f || woLocal.z <= 0.f) return Colour(0.f, 0.f, 0.f);
		Vec3 wr(-woLocal.x, -woLocal.y, woLocal.z);

		// e, ks and kd
		float e = alphaToPhongExponent();
		float ks = ShadingHelper::fresnelDielectric(woLocal.z, intIOR, extIOR);
		float kd = 1.f - ks;

		// PhongBSDF = kd * DiffuseBSDF + ks * GlossyBSDF
		float glossyBSDFEval = ((e + 2.f) / (2.f * M_PI)) * powf(std::max(Dot(wiLocal, wr), 0.f), e);
		Colour diffuseBSDF = albedo->sample(shadingData.tu, shadingData.tv) * M_1_PI;
		Colour glossyBSDF(glossyBSDFEval, glossyBSDFEval, glossyBSDFEval);
		return diffuseBSDF * kd + glossyBSDF * ks;
	}

	float PDF(const ShadingData& shadingData, const Vec3& wi) {
		Vec3 wiLocal = shadingData.frame.toLocal(wi);
		Vec3 woLocal = shadingData.frame.toLocal(shadingData.wo);

		if (wiLocal.z <= 0.f || woLocal.z <= 0.f) return 0.f;
		Vec3 wr(-woLocal.x, -woLocal.y, woLocal.z);

		// e, ks and kd
		float e = alphaToPhongExponent();
		float ks = ShadingHelper::fresnelDielectric(woLocal.z, intIOR, extIOR);
		float kd = 1.f - ks;

		// PDF
		return kd * (wiLocal.z * M_1_PI) + ks * ((e + 1.f) / (2.f * M_PI)) * powf(std::max(Dot(wiLocal, wr), 0.f), e);
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
// Refraction has been selected, and used Beer's Law for LayeredBSDF
// car2 scene can be opened to view the LayeredBSDF effects
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
		Vec3 wo = shadingData.frame.toLocal(shadingData.wo);
		Vec3 wi = shadingData.frame.toLocal(base->sample(shadingData, sampler, reflectedColour, pdf));

		if (pdf <= 0.f || wi.z == 0.f || wo.z == 0.f) {
			pdf = 0.f;
			reflectedColour = Colour(0, 0, 0);
			return Vec3(0.f, 0.f, 0.f);
		}
		
		// Handle absorbing medium
		// Since on local space, Dot(w, n) becomes w.z
		float distWo = thickness / fabs(wo.z);
		float distWi = thickness / fabs(wi.z);

		// From Beer's Law
		Colour TrWo(powf(M_E, -distWo * sigmaa.r), powf(M_E, -distWo * sigmaa.g), powf(M_E, -distWo * sigmaa.b));
		Colour TrWi(powf(M_E, -distWi * sigmaa.r), powf(M_E, -distWi * sigmaa.g), powf(M_E, -distWi * sigmaa.b));

		// Evaluate Fresnel
		float fresnelWo = ShadingHelper::fresnelDielectric(wo.z, intIOR, extIOR);
		float fresnelWi = ShadingHelper::fresnelDielectric(wi.z, intIOR, extIOR);

		float etaWo = (wo.z > 0.f) ? (extIOR / intIOR) : (intIOR / extIOR);
		float etaWi = (wi.z > 0.f) ? (extIOR / intIOR) : (intIOR / extIOR);

		// Refract
		float refractWo = 1.f - fresnelWo;
		float refractWi = 1.f - fresnelWi;

		reflectedColour = reflectedColour * ((TrWo * etaWo * refractWo) + (TrWi * etaWi * refractWi));
		return shadingData.frame.toWorld(wi);
	}

	Colour evaluate(const ShadingData& shadingData, const Vec3& wi) {
		// Add code for evaluation of layer
		Vec3 wo = shadingData.frame.toLocal(shadingData.wo);
		Vec3 wiLocal = shadingData.frame.toLocal(wi);

		if (wiLocal.z == 0.f || wo.z == 0.f) return Colour(0.f, 0.f, 0.f);
		
		// Handle absorbing medium
		// Since on local space, Dot(w, n) becomes w.z
		float distWo = thickness / fabs(wo.z);
		float distWi = thickness / fabs(wiLocal.z);

		// From Beer's Law
		Colour TrWo(powf(M_E, -distWo * sigmaa.r), powf(M_E, -distWo * sigmaa.g), powf(M_E, -distWo * sigmaa.b));
		Colour TrWi(powf(M_E, -distWi * sigmaa.r), powf(M_E, -distWi * sigmaa.g), powf(M_E, -distWi * sigmaa.b));

		// Evaluate Fresnel
		float fresnelWo = ShadingHelper::fresnelDielectric(wo.z, intIOR, extIOR);
		float fresnelWi = ShadingHelper::fresnelDielectric(wiLocal.z, intIOR, extIOR);

		float etaWo = (wo.z > 0.f) ? (extIOR / intIOR) : (intIOR / extIOR);
		float etaWi = (wiLocal.z > 0.f) ? (extIOR / intIOR) : (intIOR / extIOR);

		// Refraction
		float refractWo = 1.f - fresnelWo;
		float refractWi = 1.f - fresnelWi;

		return base->evaluate(shadingData, shadingData.frame.toWorld(wiLocal)) * ((TrWo * etaWo * refractWo) + (TrWi * etaWi * refractWi));
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