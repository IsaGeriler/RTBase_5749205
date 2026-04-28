#pragma once

// For Moller-Trumbore Ray-Triangle Intersect
#define EPSILON 0.001f
#define MOLLER_EPSILON 1e-6f

// For BVH (Bounding Volume Hierarchy)
#define MAXNODE_TRIANGLES 8
#define BOUNDS_COST 1.0f
#define INTERSECT_COST 1.5f
#define BUILD_BINS 32

#include <utility>
#include <vector>

#include "Core.h"
#include "Sampling.h"

class Ray {
public:
	// Origin, direction, and inverse direction
	Vec3 o;
	Vec3 dir;
	Vec3 invDir;

	Ray() {}
	Ray(Vec3 _o, Vec3 _d) { init(_o, _d); }

	void init(Vec3 _o, Vec3 _d) {
		o = _o;
		dir = _d;
		invDir = Vec3(1.f / dir.x, 1.f / dir.y, 1.f / dir.z);
	}

	// Return the ray at t
	Vec3 at(const float t) const { return (o + (dir * t)); }
};

class Plane {
public:
	Vec3 n;
	float d;

	void init(Vec3& _n, float _d) {
		n = _n;
		d = _d;
	}

	// Add code here
	bool rayIntersect(Ray& r, float& t) {
		float denom = Dot(n, r.dir);
		if (denom == 0) return false;
		t = -(Dot(n, r.o) + d) / denom;
		return t >= 0.f;
	}
};

class Triangle {
public:
	Vertex vertices[3];
	Vec3 e1;	 // Edge 1
	Vec3 e2;	 // Edge 2
	Vec3 n;		 // Geometric Normal
	float area;  // Triangle area
	float d;	 // For ray triangle if needed
	unsigned int materialIndex;

	void init(Vertex v0, Vertex v1, Vertex v2, unsigned int _materialIndex) {
		materialIndex = _materialIndex;
		vertices[0] = v0;
		vertices[1] = v1;
		vertices[2] = v2;
		e1 = vertices[2].p - vertices[1].p;
		e2 = vertices[0].p - vertices[2].p;
		n = e1.cross(e2).normalize();
		area = e1.cross(e2).length() * 0.5f;
		d = Dot(n, vertices[0].p);
	}

	Vec3 centre() const {
		return (vertices[0].p + vertices[1].p + vertices[2].p) / 3.0f;
	}

	// Add code here
	bool rayIntersect(const Ray& r, float& t, float& u, float& v) const {
		// Moller-Trumbore Update
		// Recalculate triangle edge coordinates
		// As Moller-Trumbore requires v (beta) and w (gamma) but we have u (alpha) and v (beta)
		Vec3 _e1 = vertices[1].p - vertices[0].p;
		Vec3 _e2 = vertices[2].p - vertices[0].p;

		Vec3 p = Cross(r.dir, _e2);  // Cross(_e2, -r.dir);
		float det = Dot(_e1, p);
		if (fabs(det) < MOLLER_EPSILON) return false;  // Ray is parallel to the plane
		float invDet = 1.f / det;

		// Cramer's Rule - Using determinant to solve for values 
		Vec3 T = r.o - vertices[0].p;
		v = Dot(T, p) * invDet;
		if (v < 0.f || v > 1.f) return false;

		Vec3 q = Cross(T, _e1);
		float w = Dot(r.dir, q) * invDet;
		if (w < 0.f || w > 1.f || w + v > 1.f) return false;
		u = 1.f - (v + w);

		t = Dot(_e2, q) * invDet;
		return t >= 0.f;
	}

	void interpolateAttributes(const float alpha, const float beta, const float gamma, Vec3& interpolatedNormal, float& interpolatedU, float& interpolatedV) const {
		interpolatedNormal = vertices[0].normal * alpha + vertices[1].normal * beta + vertices[2].normal * gamma;
		interpolatedNormal = interpolatedNormal.normalize();
		interpolatedU = vertices[0].u * alpha + vertices[1].u * beta + vertices[2].u * gamma;
		interpolatedV = vertices[0].v * alpha + vertices[1].v * beta + vertices[2].v * gamma;
	}

	// Add code here
	Vec3 sample(Sampler* sampler, float& pdf) {
		float r1 = sampler->next();
		float r2 = sampler->next();

		float alpha = 1.f - sqrtf(r1);
		float beta = r2 * sqrtf(r1);
		float gamma = 1.f - (alpha + beta);

		pdf = 1.f / area;
		return (vertices[0].p * alpha) + (vertices[1].p * beta) + (vertices[2].p * gamma);
	}

	Vec3 gNormal() {
		return (n * (Dot(vertices[0].normal, n) > 0 ? 1.f : -1.0f));
	}
};

class AABB {
public:
	// Minimum and maximum bounds
	Vec3 max;
	Vec3 min;

	AABB() { reset(); }

	void reset() {
		max = Vec3(-FLT_MAX, -FLT_MAX, -FLT_MAX);
		min = Vec3(FLT_MAX, FLT_MAX, FLT_MAX);
	}

	void extend(const Vec3 p) {
		max = Max(max, p);
		min = Min(min, p);
	}

	// Add code here
	bool rayAABB(const Ray& r, float& t) {
		Vec3 tmin = (min - r.o) * r.invDir;
		Vec3 tmax = (max - r.o) * r.invDir;

		Vec3 tentry = Min(tmin, tmax);
		Vec3 texit = Max(tmin, tmax);

		float tentry_flt = std::max(tentry.x, std::max(tentry.y, tentry.z));
		float texit_flt = std::min(texit.x, std::min(texit.y, texit.z));

		t = std::min(tentry_flt, texit_flt);
		return (tentry_flt <= texit_flt) && (texit_flt >= 0.f);
	}

	// Add code here
	bool rayAABB(const Ray& r) {
		Vec3 tmin = (min - r.o) * r.invDir;
		Vec3 tmax = (max - r.o) * r.invDir;

		Vec3 tentry = Min(tmin, tmax);
		Vec3 texit = Max(tmin, tmax);

		float tentry_flt = std::max(tentry.x, std::max(tentry.y, tentry.z));
		float texit_flt = std::min(texit.x, std::min(texit.y, texit.z));

		return (tentry_flt <= texit_flt) && (texit_flt >= 0.f);
	}

	// Add code here
	float area() {
		Vec3 size = max - min;
		return ((size.x * size.y) + (size.y * size.z) + (size.x * size.z)) * 2.0f;
	}
};

class Sphere {
public:
	Vec3 centre;
	float radius;
	
	void init(Vec3& _centre, float _radius) {
		centre = _centre;
		radius = _radius;
	}

	// Add code here
	bool rayIntersect(Ray& r, float& t) {
		Vec3 l = r.o - centre;
		float b = 2 * Dot(l, r.dir);
		float c = l.lengthSq() - powf(radius, 2);
		float dis = powf(b, 2) - c;

		// No solutions
		if (dis < 0.f) return false;

		// One real solution
		if (dis == 0.f) t = -b;

		// Two solutions
		if (dis > 0.f) t = std::min(-b - sqrtf(dis), -b + sqrtf(dis));
		return true;
	}
};

struct IntersectionData {
	unsigned int ID;
	float t;
	float alpha;
	float beta;
	float gamma;
};

// Adapted from: https://jacco.ompf2.com/2022/04/13/how-to-build-a-bvh-part-1-basics/
//				 https://jacco.ompf2.com/2022/04/18/how-to-build-a-bvh-part-2-faster-rays/
//				 https://jacco.ompf2.com/2022/04/21/how-to-build-a-bvh-part-3-quick-builds/
struct SAHBins {
	AABB bounds;
	unsigned int triangleCount = 0;
};

class BVHNode {
private:
	bool isLeafNode() const { return l == NULL && r == NULL; }

	// Calculate AABB Bounds
	void calculateBounds(std::vector<Triangle>& triangles, std::vector<unsigned int>& triangleIndexes) {
		for (unsigned int i = offset; i < offset + num; i++) {
			unsigned int triangleIdx = triangleIndexes[i];
			bounds.extend(triangles[triangleIdx].vertices[0].p);
			bounds.extend(triangles[triangleIdx].vertices[1].p);
			bounds.extend(triangles[triangleIdx].vertices[2].p);
		}
	}

	// Calculate Binned SAH Split Planes
	float bestSAHSplitPlane(std::vector<Triangle>& triangles, std::vector<unsigned int>& triangleIndexes, int& axis, float& splitPos) {
		float bestCost = FLT_MAX;
		float parentArea = bounds.area();
		for (int ax = 0; ax < 3; ax++) {
			float boundsMin = FLT_MAX;
			float boundsMax = -FLT_MAX;

			for (unsigned int i = offset; i < offset + num; i++) {
				boundsMin = std::min(boundsMin, triangles[triangleIndexes[i]].centre().coords[ax]);
				boundsMax = std::max(boundsMax, triangles[triangleIndexes[i]].centre().coords[ax]);
			}
			if (boundsMin == boundsMax) continue;

			// Populate bins
			SAHBins bins[BUILD_BINS];
			float scale = BUILD_BINS / (boundsMax - boundsMin);
			for (unsigned int i = offset; i < offset + num; i++) {
				int triangleIdx = triangleIndexes[i];
				// Clamp the bin index
				int binIdx = std::min(BUILD_BINS - 1, (int)((triangles[triangleIdx].centre().coords[ax] - boundsMin) * scale));
				binIdx = std::max(binIdx, 0);

				bins[binIdx].triangleCount++;
				bins[binIdx].bounds.extend(triangles[triangleIdx].vertices[0].p);
				bins[binIdx].bounds.extend(triangles[triangleIdx].vertices[1].p);
				bins[binIdx].bounds.extend(triangles[triangleIdx].vertices[2].p);
			}

			// Gather Data between the bins
			AABB leftBBox, rightBBox;
			float leftArea[BUILD_BINS - 1]{}, rightArea[BUILD_BINS - 1]{};
			int leftCount[BUILD_BINS - 1]{}, rightCount[BUILD_BINS - 1]{};
			int leftSum = 0, rightSum = 0;

			for (int i = 0; i < BUILD_BINS - 1; i++) {
				leftSum += bins[i].triangleCount;
				leftCount[i] = leftSum;

				if (bins[i].triangleCount > 0) {
					leftBBox.extend(bins[i].bounds.min);
					leftBBox.extend(bins[i].bounds.max);
					leftArea[i] = leftBBox.area();
				} else {
					leftArea[i] = 0;
				}

				rightSum += bins[BUILD_BINS - 1 - i].triangleCount;
				rightCount[BUILD_BINS - 2 - i] = rightSum;

				if (bins[BUILD_BINS - 1 - i].triangleCount > 0) {
					rightBBox.extend(bins[BUILD_BINS - 1 - i].bounds.min);
					rightBBox.extend(bins[BUILD_BINS - 1 - i].bounds.max);
					rightArea[BUILD_BINS - 2 - i] = rightBBox.area();
				} else {
					rightArea[BUILD_BINS - 2 - i] = 0;
				}
			}

			// Calculate SAH cost
			scale = (boundsMax - boundsMin) / BUILD_BINS;
			for (int i = 0; i < BUILD_BINS - 1; i++) {
				// Csplit = Cbounds + ((LeftAABBArea / ParentAABBArea) * LeftPrimCount * Cisect) + ((RightAABBArea / ParentAABBArea) * RightPrimCount * Cisect)
				float cost = BOUNDS_COST + ((leftArea[i] / parentArea) * leftCount[i] * INTERSECT_COST) + (rightArea[i] / parentArea) * rightCount[i] * INTERSECT_COST;
				if (cost < bestCost) {
					splitPos = boundsMin + scale * (i + 1);
					axis = ax;
					bestCost = cost;
				}
			}
		}
		return bestCost;
	}

	// Build sub-trees
	void subdivide(std::vector<Triangle>& triangles, std::vector<unsigned int>& triangleIndexes) {
		// Calculate bounds
		calculateBounds(triangles, triangleIndexes);

		// Recursion termination
		if (num <= MAXNODE_TRIANGLES) return;

		// SAH Split and Triangle Reordering (on an index array)
		int axis = -1;
		float splitPos = -FLT_MAX;
		float splitCost = bestSAHSplitPlane(triangles, triangleIndexes, axis, splitPos);
		float parentCost = num * INTERSECT_COST;  // Calculate Parent Node SAH cost
		
		// Terminate Recursion
		if (splitCost >= parentCost || axis == -1) return;

		// Reorder Triangles
		int i = offset;
		int j = i + num - 1;

		while (i <= j) {
			if (triangles[triangleIndexes[i]].centre().coords[axis] < splitPos) i++;
			else std::swap(triangleIndexes[i], triangleIndexes[j--]);
		}

		int leftCount = i - offset;
		if (leftCount == 0 || leftCount == num) return;

		// Initialize Child Nodes
		l = new BVHNode();
		r = new BVHNode();

		l->offset = offset;
		l->num = leftCount;
		
		r->offset = i;
		r->num = num - leftCount;

		num = 0;

		l->subdivide(triangles, triangleIndexes);
		r->subdivide(triangles, triangleIndexes);
	}
public:
	AABB bounds;
	BVHNode* r;
	BVHNode* l;
	// This can store an offset and number of triangles in a global triangle list for example
	// But you can store this however you want!
	unsigned int offset = 0;
	unsigned int num = 0;
	
	BVHNode() {
		r = NULL;
		l = NULL;
	}

	~BVHNode() {
		if (r != NULL) delete r;
		if (l != NULL) delete l;
	}

	// Note there are several options for how to implement the build method. Update this as required
	void build(std::vector<Triangle>& inputTriangles, std::vector<unsigned int>& inputTriangleIndexes) {
		// Add BVH building code here
		// Update number of primitives in the root, otherwise subdivide will be immediately terminated
		num = inputTriangleIndexes.size();

		// Recursive BVH Building (by subdividing the root)
		subdivide(inputTriangles, inputTriangleIndexes);
	}

	void traverse(const Ray& ray, const std::vector<Triangle>& triangles, const std::vector<unsigned int>& triangleIndexes, IntersectionData& intersection) {
		// Add BVH Traversal code here
		// Check Ray-AABB Intersection
		if (!bounds.rayAABB(ray)) return;
		// If node is leaf, check primitives - Else, check the childs
		if (isLeafNode()) {
			for (unsigned int i = offset; i < offset + num; i++) {
				float t, u, v;
				if (triangles[triangleIndexes[i]].rayIntersect(ray, t, u, v)) {
					if (t < intersection.t) {
						intersection.ID = triangleIndexes[i];
						intersection.t = t;
						intersection.alpha = u;
						intersection.beta = v;
						intersection.gamma = 1.f - (u + v);
					}
				}
			}
		} else {
			if (l != NULL) l->traverse(ray, triangles, triangleIndexes, intersection);
			if (r != NULL) r->traverse(ray, triangles, triangleIndexes, intersection);
		}
	}

	IntersectionData traverse(const Ray& ray, const std::vector<Triangle>& triangles, const std::vector<unsigned int>& triangleIndexes) {
		IntersectionData intersection;
		intersection.t = FLT_MAX;
		traverse(ray, triangles, triangleIndexes, intersection);
		return intersection;
	}

	bool traverseVisible(const Ray& ray, const std::vector<Triangle>& triangles, const std::vector<unsigned int>& triangleIndexes, const float maxT) {
		// Add visibility code here
		float t;
		if (!bounds.rayAABB(ray, t) || t > maxT) return true;

		if (isLeafNode()) {
			float u, v;
			for (unsigned int i = offset; i < offset + num; i++) {
				// Terminate at the first intersect
				if (triangles[triangleIndexes[i]].rayIntersect(ray, t, u, v) && t <= maxT) return false;
			}
			return true;
		}
		if (l != NULL && !l->traverseVisible(ray, triangles, triangleIndexes, maxT)) return false;
		if (r != NULL && !r->traverseVisible(ray, triangles, triangleIndexes, maxT)) return false;
		return true;
	}
};