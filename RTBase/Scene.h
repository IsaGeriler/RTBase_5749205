#pragma once

#include "Core.h"
#include "Geometry.h"
#include "Imaging.h"
#include "Lights.h"
#include "Materials.h"
#include "Sampling.h"

class Camera {
public:
	Matrix projectionMatrix;		 // Camera -> Clip Space (P)
	Matrix inverseProjectionMatrix;  // Clip Space -> Camera (inverse P)
	Matrix camera;					 // Camera -> World (inverse V)
	Matrix cameraToView;			 // World -> Camera (V)

	float width = 0.f;
	float height = 0.f;
	float Afilm;
	
	Vec3 origin;		 // Ray Origin
	Vec3 viewDirection;
	
	void init(Matrix ProjectionMatrix, int screenwidth, int screenheight) {
		projectionMatrix = ProjectionMatrix;
		inverseProjectionMatrix = ProjectionMatrix.invert();
		width = (float)screenwidth;
		height = (float)screenheight;
		float Wlens = (2.f / ProjectionMatrix.a[1][1]);
		float aspect = ProjectionMatrix.a[0][0] / ProjectionMatrix.a[1][1];
		float Hlens = Wlens * aspect;
		Afilm = Wlens * Hlens;
	}

	void updateView(Matrix V) {
		camera = V;
		cameraToView = V.invert();
		origin = camera.mulPoint(Vec3(0.f, 0.f, 0.f));
		viewDirection = inverseProjectionMatrix.mulPointAndPerspectiveDivide(Vec3(0.f, 0.f, 1.f));
		viewDirection = camera.mulVec(viewDirection);
		viewDirection = viewDirection.normalize();
	}

	// Add code here
	Ray generateRay(float x, float y) {
		// Normalize points to NDC
		float xc = (2.f * x / width) - 1.f;
		float yc = (2.f * (1.f - (y / height))) - 1.f;

		// NDC to Clip Space
		Vec3 pclip(xc, yc, 0.f, 1.f);

		// Clip Space to Camera Space
		Vec3 dCamera = inverseProjectionMatrix.mulPointAndPerspectiveDivide(pclip);

		// Camera Space to World Space
		Vec3 dir = camera.mulVec(dCamera).normalize();
		return Ray(origin, dir);
	}

	bool projectOntoCamera(const Vec3& p, float& x, float& y) {
		Vec3 pview = cameraToView.mulPoint(p);
		Vec3 pproj = projectionMatrix.mulPointAndPerspectiveDivide(pview);
		x = (pproj.x + 1.f) * 0.5f;
		y = (pproj.y + 1.f) * 0.5f;
		if (x < 0.f || x > 1.f || y < 0.f || y > 1.f) return false;
		x = x * width;
		y = 1.f - y;
		y = y * height;
		return true;
	}
};

class Scene {
public:
	std::vector<Triangle> triangles;
	std::vector<unsigned int> triangleIndexes;
	std::vector<BSDF*> materials;
	std::vector<Light*> lights;

	Light* background = NULL;
	BVHNode* bvh = NULL;
	Camera camera;
	AABB bounds;
	
	void build() {
		// Add BVH building code here
		bvh = new BVHNode();
		bvh->build(triangles, triangleIndexes);
		// Do not touch the code below this line!
		// Build light list
		for (int i = 0; i < triangles.size(); i++) {
			if (materials[triangles[i].materialIndex]->isLight()) {
				AreaLight* light = new AreaLight();
				light->triangle = &triangles[i];
				light->emission = materials[triangles[i].materialIndex]->emission;
				lights.push_back(light);
			}
		}
	}

	IntersectionData traverse(const Ray& ray) {
		return bvh->traverse(ray, triangles, triangleIndexes);
	}

	Light* sampleLight(Sampler* sampler, float& pmf) {
		pmf = 1.f / lights.size();
		unsigned int index = std::min((unsigned int)floor(sampler->next() * lights.size()), (unsigned int)(lights.size() - 1));
		return lights[index];
	}

	// Do not modify any code below this line
	void init(std::vector<Triangle> meshTriangles, std::vector<BSDF*> meshMaterials, Light* _background) {
		for (int i = 0; i < meshTriangles.size(); i++) {
			// Save the original index positions, for optimising, to perform index swapping in the BVH
			// As the inputTriangles contain more data, e.g. material index, it will be more costly
			triangles.push_back(meshTriangles[i]);
			triangleIndexes.push_back(i);
			bounds.extend(meshTriangles[i].vertices[0].p);
			bounds.extend(meshTriangles[i].vertices[1].p);
			bounds.extend(meshTriangles[i].vertices[2].p);
		}

		for (int i = 0; i < meshMaterials.size(); i++) {
			materials.push_back(meshMaterials[i]);
		}

		background = _background;
		if (background->totalIntegratedPower() > 0) {
			lights.push_back(background);
		}
	}
	
	bool visible(const Vec3& p1, const Vec3& p2) {
		Ray ray;
		Vec3 dir = p2 - p1;
		float maxT = dir.length() - (2.0f * EPSILON);
		dir = dir.normalize();
		ray.init(p1 + (dir * EPSILON), dir);
		return bvh->traverseVisible(ray, triangles, triangleIndexes, maxT);
	}

	Colour emit(Triangle* light, ShadingData shadingData, Vec3 wi) {
		return materials[light->materialIndex]->emit(shadingData, wi);
	}

	ShadingData calculateShadingData(IntersectionData intersection, Ray& ray) {
		ShadingData shadingData = {};
		if (intersection.t < FLT_MAX) {
			shadingData.x = ray.at(intersection.t);
			shadingData.gNormal = triangles[intersection.ID].gNormal();
			triangles[intersection.ID].interpolateAttributes(intersection.alpha, intersection.beta, intersection.gamma, shadingData.sNormal, shadingData.tu, shadingData.tv);
			shadingData.bsdf = materials[triangles[intersection.ID].materialIndex];
			shadingData.wo = -ray.dir;
			if (shadingData.bsdf->isTwoSided()) {
				if (Dot(shadingData.wo, shadingData.sNormal) < 0) {
					shadingData.sNormal = -shadingData.sNormal;
				}
				if (Dot(shadingData.wo, shadingData.gNormal) < 0) {
					shadingData.gNormal = -shadingData.gNormal;
				}
			}
			shadingData.frame.fromVector(shadingData.sNormal);
			shadingData.t = intersection.t;
		} else {
			shadingData.wo = -ray.dir;
			shadingData.t = intersection.t;
		}
		return shadingData;
	}
};