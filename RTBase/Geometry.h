#pragma once

#include <algorithm>
#include <cmath>
#include <iterator>
#include <vector>
#include <utility>

#include "Core.h"
#include "Sampling.h"

class Ray {
public:
	Vec3 o;
	Vec3 dir;
	Vec3 invDir;
	
	Ray() {}
	Ray(Vec3 _o, Vec3 _d) { init(_o, _d); }
	
	void init(Vec3 _o, Vec3 _d) {
		o = _o;
		dir = _d;
		invDir = Vec3(1.0f / dir.x, 1.0f / dir.y, 1.0f / dir.z);
	}
	
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
		t = -(Dot(n, r.o) + d) / Dot(n, r.dir);
		return t >= 0.f;
	}
};

#define EPSILON 1e-6

class Triangle {
public:
	Vertex vertices[3];
	Vec3 e0;	 // Edge 0
	Vec3 e1;	 // Edge 1
	Vec3 e2;	 // Edge 2
	Vec3 n;		 // Geometric Normal
	float area;  // Triangle area
	float d;     // For ray triangle if needed
	unsigned int materialIndex;
	
	void init(Vertex v0, Vertex v1, Vertex v2, unsigned int _materialIndex) {
		materialIndex = _materialIndex;
		vertices[0] = v0;
		vertices[1] = v1;
		vertices[2] = v2;
		e0 = vertices[1].p - vertices[0].p;
		e1 = vertices[2].p - vertices[1].p;
		e2 = vertices[0].p - vertices[2].p;
		n = e1.cross(e2).normalize();
		area = e1.cross(e2).length() * 0.5f;
		d = Dot(n, vertices[0].p);
	}

	Vec3 centre() const { return (vertices[0].p + vertices[1].p + vertices[2].p) / 3.0f; }

	// Add code here
	bool rayIntersect(const Ray& r, float& t, float& u, float& v) const {
		// Find Ray-Plane Intersaction First
		float denominator = Dot(n, r.dir);
		if (denominator == 0) return false;
		t = -(Dot(n, r.o) + d) / denominator;
		if (t < 0.f) return false;

		// Then Find Ray-Triangle Intersaction, by finding u (alpha) and v (beta)
		// -- Find u
		Vec3 q1 = r.at(t) - vertices[1].p;
		Vec3 C1 = Cross(e1, q1);
		float invArea = 1.f / (area * 2.f);
		u = Dot(C1, n) * invArea;
		if (u < 0.f || u > 1.f) return false;
		
		// -- Find v
		Vec3 q2 = r.at(t) - vertices[2].p;
		Vec3 C2 = Cross(e2, q2);
		v = Dot(C2, n) * invArea;
		if (v < 0.f || v > 1.f || u + v > 1.f) return false;
		return true;
	}

	bool rayIntersectMollerTrumbore(const Ray& r, float& t, float& u, float& v) const {
		// Recalculate the triangle edge coordinates
		// As Möller-Trumbore requires v (beta) and w (gamma) but we have u (alpha) and v (beta)
		Vec3 _e1 = vertices[1].p - vertices[0].p;
		Vec3 _e2 = vertices[2].p - vertices[0].p;

		Vec3 p = Cross(r.dir, _e2);  // Cross(_e2, -r.dir);
		float det = Dot(_e1, p);
		if (fabs(det) < EPSILON) return false;  // Ray is parallel to plane
		float invDet = 1.f / det;

		// Cramer's Rule - Using determinant to solve for values 
		Vec3 T = r.o - vertices[0].p;
		u = Dot(T, p) * invDet;
		if (u < 0.f || u > 1.f) return false;

		Vec3 q = Cross(T, _e1);
		v = Dot(r.dir, q) * invDet;
		if (v < 0.f || v > 1.f || u + v > 1.f) return false;

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
		pdf = 1 / area;

		float r1 = sampler->next();
		float r2 = sampler->next();

		float alpha = 1 - sqrtf(r1);
		float beta = r2 * sqrtf(r1);
		float gamma = 1 - (alpha + beta);

		return vertices[0].p * alpha + vertices[1].p * beta + vertices[2].p * gamma;
	}

	Vec3 gNormal() {
		return (n * (Dot(vertices[0].normal, n) > 0 ? 1.0f : -1.0f));
	}
};

class AABB {
public:
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

		float t_entry = std::max(tentry.x, std::max(tentry.y, tentry.z));
		float t_exit = std::min(texit.x, std::min(texit.y, texit.z));
		t = std::min(t_entry, t_exit);
		return (t_entry <= t_exit) && (t_exit > 0.f);
	}

	// Add code here
	bool rayAABB(const Ray& r) {
		Vec3 tmin = (min - r.o) * r.invDir;
		Vec3 tmax = (max - r.o) * r.invDir;

		Vec3 tentry = Min(tmin, tmax);
		Vec3 texit = Max(tmin, tmax);

		float t_entry = std::max(tentry.x, std::max(tentry.y, tentry.z));
		float t_exit = std::min(texit.x, std::min(texit.y, texit.z));
		return (t_entry <= t_exit) && (t_exit > 0.f);
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
		float c = Dot(l, l) - (radius * radius);
		float dis = b * b - c;  // a = 1

		// No solutions
		if (dis < 0.f) return false;

		// One real solution
		if (dis == 0.f) t = -b;

		// Two real solution
		if (dis > 0.f) {
			float root1 = -b - sqrtf(dis);
			float root2 = -b + sqrtf(dis);
			t = root1 > 0.f && root2 > 0.f ? std::min(root1, root2) :
				(root1 > 0.f ? root1 : root2);
		}
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

#define MAXNODE_TRIANGLES 8
#define TRAVERSE_COST 1.0f
#define TRIANGLE_COST 2.0f
#define BUILD_BINS 32

// Adapted from: https://jacco.ompf2.com/2022/04/13/how-to-build-a-bvh-part-1-basics/
// will also adapt more from:
// https://jacco.ompf2.com/2022/04/18/how-to-build-a-bvh-part-2-faster-rays/
// https://jacco.ompf2.com/2022/04/21/how-to-build-a-bvh-part-3-quick-builds/
// https://www.sci.utah.edu/~wald/Publications/2007/ParallelBVHBuild/fastbuild.pdf
class BVHNode {
private:
	bool isLeaf() const { return l == nullptr && r == nullptr; }

	void calculateBounds(std::vector<Triangle>& triangles) {
		for (unsigned int i = 0; i < used; i++) {
			bounds.extend(triangles[offset + i].vertices[0].p);
			bounds.extend(triangles[offset + i].vertices[1].p);
			bounds.extend(triangles[offset + i].vertices[2].p);
		}
	}

	void subdivide(std::vector<Triangle>& triangles) {
		if (used <= 2) return;

		Vec3 extent = bounds.max - bounds.min;
		unsigned int axis = 0;
		if (extent.y > extent.x) axis = 1;
		if (extent.z > extent[axis]) axis = 2;
		float split = bounds.min[axis] + extent[axis] * 0.5f;

		int i = offset;
		int j = i + used - 1;

		while (i <= j) {
			if (triangles[i].centre()[axis] < split) i++;
			else std::swap(triangles[i], triangles[j--]);
		}

		int leftCount = i - offset;
		if (leftCount == 0 || leftCount == used) return;

		l = new BVHNode();
		r = new BVHNode();

		l->offset = offset;
		l->used = leftCount;

		r->offset = i;
		r->used = used - leftCount;

		used = 0;

		l->calculateBounds(triangles);
		r->calculateBounds(triangles);

		l->subdivide(triangles);
		r->subdivide(triangles);
	}
public:
	AABB bounds;
	BVHNode* r;
	BVHNode* l;
	
	// This can store an offset and number of triangles in a global triangle list for example
	// But you can store this however you want!
	
	// Store offset (first triangle index) and size in a reordered list of triangles
	unsigned int offset = 0;
	unsigned int used = 0;
	
	BVHNode() {
		r = NULL;
		l = NULL;
	}

	// Note there are several options for how to implement the build method. Update this as required
	void build(std::vector<Triangle>& inputTriangles, std::vector<Triangle>& outputTriangles) {
		// Add BVH building code here
		// Add code to calculate bounds, SAH split planes, and building sub trees
		calculateBounds(inputTriangles);
		subdivide(inputTriangles);
		std::copy(inputTriangles.begin(), inputTriangles.end(), std::back_inserter(outputTriangles));
	}

	void traverse(const Ray& ray, const std::vector<Triangle>& triangles, IntersectionData& intersection) {
		// Add BVH Traversal code here
		// Check bounds for whether the ray intersects AABB
		if (!bounds.rayAABB(ray)) return;
		
		if (!isLeaf()) {
			// If not leaf, traverse children nodes
			if (l != nullptr) l->traverse(ray, triangles, intersection);
			if (r != nullptr) r->traverse(ray, triangles, intersection);
		} else {
			// Else ray-triangle for all triangles in node
			float t, u, v;
			for (unsigned int i = 0; i < triangles.size(); i++) {
				if (triangles[i].rayIntersectMollerTrumbore(ray, t, u, v)) {
					if (t < intersection.t) {
						intersection.ID = i;
						intersection.t = t;
						intersection.alpha = u;
						intersection.beta = v;
						intersection.gamma = 1.f - u - v;
					}
				}
			}
		}
	}

	IntersectionData traverse(const Ray& ray, const std::vector<Triangle>& triangles) {
		IntersectionData intersection;
		intersection.t = FLT_MAX;
		traverse(ray, triangles, intersection);
		return intersection;
	}

	bool traverseVisible(const Ray& ray, const std::vector<Triangle>& triangles, const float maxT) {
		// Add visibility code here, similar to traverse, but return false as soon as a primitive is intersected
		// Check bounds for whether the ray intersects AABB
		float t;
		if (!bounds.rayAABB(ray, t) || t > maxT) return true;

		if (!isLeaf()) {
			// If not leaf, traverse children nodes
			if (l != nullptr) l->traverseVisible(ray, triangles, maxT);
			if (r != nullptr) r->traverseVisible(ray, triangles, maxT);
		} else {
			// Else ray-triangle for all triangles in node
			float t, u, v;
			for (unsigned int i = 0; i < triangles.size(); i++) {
				if (triangles[i].rayIntersectMollerTrumbore(ray, t, u, v)) return false;
			}
			return true;
		}
	}
};