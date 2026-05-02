#pragma once

// Do not change this code!

#define _USE_MATH_DEFINES
#define SQ(x) (x * x)

#include <algorithm>
#include <math.h>
#include <memory.h>
#include <vector>

// Stop warnings about M_PI being a double
#pragma warning( disable : 4244)

// Primes Vector Declaration for using at Halton Sampler
const std::vector<int> primes = { 2, 3, 5, 7, 11, 13, 17, 19, 23, 29, 31, 37, 41, 43, 47, 53, 59, 61, 67, 71,
	73, 79, 83, 89, 97, 101, 103, 107, 109, 113, 127, 131, 137, 139, 149, 151, 157, 163, 167, 173,
	179, 181, 191, 193, 197, 199, 211, 223, 227, 229, 233, 239, 241, 251, 257, 263, 269, 271, 277, 281,
	283, 293, 307, 311, 313, 317, 331, 337, 347, 349, 353, 359, 367, 373, 379, 383, 389, 397, 401, 409,
	419, 421, 431, 433, 439, 443, 449, 457, 461, 463, 467, 479, 487, 491, 499, 503, 509, 521, 523, 541,
	547, 557, 563, 569, 571, 577, 587, 593, 599, 601, 607, 613, 617, 619, 631, 641, 643, 647, 653, 659,
	661, 673, 677, 683, 691, 701, 709, 719, 727, 733, 739, 743, 751, 757, 761, 769, 773, 787, 797, 809,
	811, 821, 823, 827, 829, 839, 853, 857, 859, 863, 877, 881, 883, 887, 907, 911, 919, 929, 937, 941,
	947, 953, 967, 971, 977, 983, 991, 997, 1009, 1013, 1019, 1021, 1031, 1033, 1039, 1049, 1051, 1061, 1063, 1069,
	1087, 1091, 1093, 1097, 1103, 1109, 1117, 1123, 1129, 1151, 1153, 1163, 1171, 1181, 1187, 1193, 1201, 1213, 1217, 1223,
	1229, 1231, 1237, 1249, 1259, 1277, 1279, 1283, 1289, 1291, 1297, 1301, 1303, 1307, 1319, 1321, 1327, 1361, 1367, 1373,
	1381, 1399, 1409, 1423, 1427, 1429, 1433, 1439, 1447, 1451, 1453, 1459, 1471, 1481, 1483, 1487, 1489, 1493, 1499, 1511,
	1523, 1531, 1543, 1549, 1553, 1559, 1567, 1571, 1579, 1583, 1597, 1601, 1607, 1609, 1613, 1619, 1621, 1627, 1637, 1657,
	1663, 1667, 1669, 1693, 1697, 1699, 1709, 1721, 1723, 1733, 1741, 1747, 1753, 1759, 1777, 1783, 1787, 1789, 1801, 1811,
	1823, 1831, 1847, 1861, 1867, 1871, 1873, 1877, 1879, 1889, 1901, 1907, 1913, 1931, 1933, 1949, 1951, 1973, 1979, 1987,
	1993, 1997, 1999, 2003, 2011, 2017, 2027, 2029, 2039, 2053, 2063, 2069, 2081, 2083, 2087, 2089, 2099, 2111, 2113, 2129,
	2131, 2137, 2141, 2143, 2153, 2161, 2179, 2203, 2207, 2213, 2221, 2237, 2239, 2243, 2251, 2267, 2269, 2273, 2281, 2287,
	2293, 2297, 2309, 2311, 2333, 2339, 2341, 2347, 2351, 2357, 2371, 2377, 2381, 2383, 2389, 2393, 2399, 2411, 2417, 2423,
	2437, 2441, 2447, 2459, 2467, 2473, 2477, 2503, 2521, 2531, 2539, 2543, 2549, 2551, 2557, 2579, 2591, 2593, 2609, 2617,
	2621, 2633, 2647, 2657, 2659, 2663, 2671, 2677, 2683, 2687, 2689, 2693, 2699, 2707, 2711, 2713, 2719, 2729, 2731, 2741,
	2749, 2753, 2767, 2777, 2789, 2791, 2797, 2801, 2803, 2819, 2833, 2837, 2843, 2851, 2857, 2861, 2879, 2887, 2897, 2903,
	2909, 2917, 2927, 2939, 2953, 2957, 2963, 2969, 2971, 2999, 3001, 3011, 3019, 3023, 3037, 3041, 3049, 3061, 3067, 3079,
	3083, 3089, 3109, 3119, 3121, 3137, 3163, 3167, 3169, 3181, 3187, 3191, 3203, 3209, 3217, 3221, 3229, 3251, 3253, 3257,
	3259, 3271, 3299, 3301, 3307, 3313, 3319, 3323, 3329, 3331, 3343, 3347, 3359, 3361, 3371, 3373, 3389, 3391, 3407, 3413,
	3433, 3449, 3457, 3461, 3463, 3467, 3469, 3491, 3499, 3511, 3517, 3527, 3529, 3533, 3539, 3541, 3547, 3557, 3559, 3571,
	3581, 3583, 3593, 3607, 3613, 3617, 3623, 3631, 3637, 3643, 3659, 3671, 3673, 3677, 3691, 3697, 3701, 3709, 3719, 3727,
	3733, 3739, 3761, 3767, 3769, 3779, 3793, 3797, 3803, 3821, 3823, 3833, 3847, 3851, 3853, 3863, 3877, 3881, 3889, 3907,
	3911, 3917, 3919, 3923, 3929, 3931, 3943, 3947, 3967, 3989, 4001, 4003, 4007, 4013, 4019, 4021, 4027, 4049, 4051, 4057,
	4073, 4079, 4091, 4093, 4099, 4111, 4127, 4129, 4133, 4139, 4153, 4157, 4159, 4177, 4201, 4211, 4217, 4219, 4229, 4231,
	4241, 4243, 4253, 4259, 4261, 4271, 4273, 4283, 4289, 4297, 4327, 4337, 4339, 4349, 4357, 4363, 4373, 4391, 4397, 4409,
	4421, 4423, 4441, 4447, 4451, 4457, 4463, 4481, 4483, 4493, 4507, 4513, 4517, 4519, 4523, 4547, 4549, 4561, 4567, 4583,
	4591, 4597, 4603, 4621, 4637, 4639, 4643, 4649, 4651, 4657, 4663, 4673, 4679, 4691, 4703, 4721, 4723, 4729, 4733, 4751,
	4759, 4783, 4787, 4789, 4793, 4799, 4801, 4813, 4817, 4831, 4861, 4871, 4877, 4889, 4903, 4909, 4919, 4931, 4933, 4937,
	4943, 4951, 4957, 4967, 4969, 4973, 4987, 4993, 4999, 5003, 5009, 5011, 5021, 5023, 5039, 5051, 5059, 5077, 5081, 5087,
	5099, 5101, 5107, 5113, 5119, 5147, 5153, 5167, 5171, 5179, 5189, 5197, 5209, 5227, 5231, 5233, 5237, 5261, 5273, 5279,
	5281, 5297, 5303, 5309, 5323, 5333, 5347, 5351, 5381, 5387, 5393, 5399, 5407, 5413, 5417, 5419, 5431, 5437, 5441, 5443,
	5449, 5471, 5477, 5479, 5483, 5501, 5503, 5507, 5519, 5521, 5527, 5531, 5557, 5563, 5569, 5573, 5581, 5591, 5623, 5639,
	5641, 5647, 5651, 5653, 5657, 5659, 5669, 5683, 5689, 5693, 5701, 5711, 5717, 5737, 5741, 5743, 5749, 5779, 5783, 5791,
	5801, 5807, 5813, 5821, 5827, 5839, 5843, 5849, 5851, 5857, 5861, 5867, 5869, 5879, 5881, 5897, 5903, 5923, 5927, 5939,
	5953, 5981, 5987, 6007, 6011, 6029, 6037, 6043, 6047, 6053, 6067, 6073, 6079, 6089, 6091, 6101, 6113, 6121, 6131, 6133,
	6143, 6151, 6163, 6173, 6197, 6199, 6203, 6211, 6217, 6221, 6229, 6247, 6257, 6263, 6269, 6271, 6277, 6287, 6299, 6301,
	6311, 6317, 6323, 6329, 6337, 6343, 6353, 6359, 6361, 6367, 6373, 6379, 6389, 6397, 6421, 6427, 6449, 6451, 6469, 6473,
	6481, 6491, 6521, 6529, 6547, 6551, 6553, 6563, 6569, 6571, 6577, 6581, 6599, 6607, 6619, 6637, 6653, 6659, 6661, 6673,
	6679, 6689, 6691, 6701, 6703, 6709, 6719, 6733, 6737, 6761, 6763, 6779, 6781, 6791, 6793, 6803, 6823, 6827, 6829, 6833,
	6841, 6857, 6863, 6869, 6871, 6883, 6899, 6907, 6911, 6917, 6947, 6949, 6959, 6961, 6967, 6971, 6977, 6983, 6991, 6997,
	7001, 7013, 7019, 7027, 7039, 7043, 7057, 7069, 7079, 7103, 7109, 7121, 7127, 7129, 7151, 7159, 7177, 7187, 7193, 7207,
	7211, 7213, 7219, 7229, 7237, 7243, 7247, 7253, 7283, 7297, 7307, 7309, 7321, 7331, 7333, 7349, 7351, 7369, 7393, 7411,
	7417, 7433, 7451, 7457, 7459, 7477, 7481, 7487, 7489, 7499, 7507, 7517, 7523, 7529, 7537, 7541, 7547, 7549, 7559, 7561,
	7573, 7577, 7583, 7589, 7591, 7603, 7607, 7621, 7639, 7643, 7649, 7669, 7673, 7681, 7687, 7691, 7699, 7703, 7717, 7723,
	7727, 7741, 7753, 7757, 7759, 7789, 7793, 7817, 7823, 7829, 7841, 7853, 7867, 7873, 7877, 7879, 7883, 7901, 7907, 7919 };

// Balance Heuristics for MIS (Multiple Importance Sampling)
float balanceHeuristics(float pj, float pk) {
	if (pj + pk <= 0.f) return 0.f;
	return (pj / (pj + pk));
}

// Power Heuristics for MIS (Multiple Importance Sampling)
// Beta is adjustable, but is usually 2
float powerHeuristics(float pj, float pk, int beta = 2) {
	if (pj + pk <= 0.f) return 0.f;
	return powf(pj, beta) / (powf(pj, beta) + powf(pk, beta));
}

class Colour {
public:
	float r;
	float g;
	float b;
	
	Colour() { r = 0; g = 0; b = 0; }
	Colour(float _r, float _g, float _b) { r = _r; g = _g; b = _b; }
	
	Colour(unsigned char _r, unsigned char _g, unsigned char _b, unsigned char _a) {
		r = (float)_r / 255.0f;
		g = (float)_g / 255.0f;
		b = (float)_b / 255.0f;
	}
	
	void ToRGB(unsigned char& cr, unsigned char& cg, unsigned char& cb) {
		cr = (unsigned char)(r * 255);
		cg = (unsigned char)(g * 255);
		cb = (unsigned char)(b * 255);
	}
	
	Colour operator+(const Colour& colour) const { Colour c; c.r = r + colour.r; c.g = g + colour.g; c.b = b + colour.b; return c; }
	Colour operator-(const Colour& colour) const { Colour c; c.r = r - colour.r; c.g = g - colour.g; c.b = b - colour.b; return c; }
	Colour operator*(const Colour& colour) const { Colour c; c.r = r * colour.r; c.g = g * colour.g; c.b = b * colour.b; return c; }
	Colour operator/(const Colour& colour) const { Colour c; c.r = r / colour.r; c.g = g / colour.g; c.b = b / colour.b; return c; }
	
	Colour operator*(const float v) const { Colour c; c.r = r * v; c.g = g * v; c.b = b * v; return c; }
	Colour operator/(const float v) const { Colour c; c.r = r / v; c.g = g / v; c.b = b / v; return c; }
	
	float Lum() { return ((0.2126f * r) + (0.7152f * g) + (0.0722f * b)); }
};

class Vec3 {
public:
	union {
		struct { float x; float y; float z; float w; };
		float coords[4];
	};

	Vec3() { x = 0.f; y = 0.f; z = 0.f; w = 1.f; }
	Vec3(float _x, float _y, float _z) { x = _x; y = _y; z = _z; w = 1.f; }
	Vec3(float _x, float _y, float _z, float _w) { x = _x; y = _y; z = _z; w = _w; }

	Vec3 operator+(const Vec3 v) const { return Vec3(x + v.x, y + v.y, z + v.z); }
	Vec3 operator-(const Vec3 v) const { return Vec3(x - v.x, y - v.y, z - v.z); }
	Vec3 operator*(const float v) const { return Vec3(x * v, y * v, z * v); }
	Vec3 operator/(const float v) const { return Vec3(x / v, y / v, z / v, w / v); }
	Vec3 operator*(const Vec3 v) const { return Vec3(x * v.x, y * v.y, z * v.z); }
	Vec3 operator-() const { return Vec3(-x, -y, -z); }
	
	float dot(Vec3 v) const { return ((x * v.x) + (y * v.y) + (z * v.z)); }
	float lengthSq() { return ((x * x) + (y * y) + (z * z)); }
	float length() { return sqrtf((x * x) + (y * y) + (z * z)); }

	Vec3 cross(Vec3 v) const { return Vec3((y * v.z) - (z * v.y), (z * v.x) - (x * v.z), (x * v.y) - (y * v.x)); }
	Vec3 perspectiveDivide() const { return Vec3(x / w, y / w, z / w, 1.0f / w); }
	
	Vec3 normalize() const {
		float l = 1.0f / sqrtf((x * x) + (y * y) + (z * z));
		return Vec3(x * l, y * l, z * l);
	}
};

static float Dot(const Vec3 v1, const Vec3 v2) { return ((v1.x * v2.x) + (v1.y * v2.y) + (v1.z * v2.z)); }
static Vec3 Cross(const Vec3& v1, const Vec3& v2) { return Vec3((v1.y * v2.z) - (v1.z * v2.y), (v1.z * v2.x) - (v1.x * v2.z), (v1.x * v2.y) - (v1.y * v2.x)); }

static Vec3 Max(Vec3 a, Vec3 b) { return Vec3(a.x > b.x ? a.x : b.x, a.y > b.y ? a.y : b.y, a.z > b.z ? a.z : b.z); }
static Vec3 Min(Vec3 a, Vec3 b) { return Vec3(a.x < b.x ? a.x : b.x, a.y < b.y ? a.y : b.y, a.z < b.z ? a.z : b.z); }

struct Vertex {
	Vec3 p;
	Vec3 normal;
	float u;
	float v;
};

class Matrix {
public:
	union {
		float a[4][4];
		float m[16];
	};

	Matrix() { identity(); }
	
	Matrix(float m00, float m01, float m02, float m03, float m10, float m11, float m12, float m13, 
		float m20, float m21, float m22, float m23, float m30, float m31, float m32, float m33) {
		a[0][0] = m00; a[0][1] = m01; a[0][2] = m02; a[0][3] = m03;
		a[1][0] = m10; a[1][1] = m11; a[1][2] = m12; a[1][3] = m13;
		a[2][0] = m20; a[2][1] = m21; a[2][2] = m22; a[2][3] = m23;
		a[3][0] = m30; a[3][1] = m31; a[3][2] = m32; a[3][3] = m33;
	}

	void identity() {
		memset(m, 0, 16 * sizeof(float));
		m[0] = 1.f; m[5] = 1.f; m[10] = 1.f; m[15] = 1.f;
	}

	Matrix transpose() {
		return Matrix(
			a[0][0], a[1][0], a[2][0], a[3][0],
			a[0][1], a[1][1], a[2][1], a[3][1],
			a[0][2], a[1][2], a[2][2], a[3][2],
			a[0][3], a[1][3], a[2][3], a[3][3]
		);
	}

	float& operator[](int index) { return m[index]; }
	
	static Matrix translation(const Vec3& v) {
		Matrix mat;
		mat.a[0][3] = v.x;
		mat.a[1][3] = v.y;
		mat.a[2][3] = v.z;
		return mat;
	}

	static Matrix scaling(const Vec3& v) {
		Matrix mat;
		mat.m[0] = v.x;
		mat.m[5] = v.y;
		mat.m[10] = v.z;
		return mat;
	}

	Matrix mul(const Matrix& matrix) const {
		Matrix ret;
		ret.m[0] = m[0] * matrix.m[0] + m[1] * matrix.m[4] + m[2] * matrix.m[8] + m[3] * matrix.m[12];
		ret.m[1] = m[0] * matrix.m[1] + m[1] * matrix.m[5] + m[2] * matrix.m[9] + m[3] * matrix.m[13];
		ret.m[2] = m[0] * matrix.m[2] + m[1] * matrix.m[6] + m[2] * matrix.m[10] + m[3] * matrix.m[14];
		ret.m[3] = m[0] * matrix.m[3] + m[1] * matrix.m[7] + m[2] * matrix.m[11] + m[3] * matrix.m[15];
		ret.m[4] = m[4] * matrix.m[0] + m[5] * matrix.m[4] + m[6] * matrix.m[8] + m[7] * matrix.m[12];
		ret.m[5] = m[4] * matrix.m[1] + m[5] * matrix.m[5] + m[6] * matrix.m[9] + m[7] * matrix.m[13];
		ret.m[6] = m[4] * matrix.m[2] + m[5] * matrix.m[6] + m[6] * matrix.m[10] + m[7] * matrix.m[14];
		ret.m[7] = m[4] * matrix.m[3] + m[5] * matrix.m[7] + m[6] * matrix.m[11] + m[7] * matrix.m[15];
		ret.m[8] = m[8] * matrix.m[0] + m[9] * matrix.m[4] + m[10] * matrix.m[8] + m[11] * matrix.m[12];
		ret.m[9] = m[8] * matrix.m[1] + m[9] * matrix.m[5] + m[10] * matrix.m[9] + m[11] * matrix.m[13];
		ret.m[10] = m[8] * matrix.m[2] + m[9] * matrix.m[6] + m[10] * matrix.m[10] + m[11] * matrix.m[14];
		ret.m[11] = m[8] * matrix.m[3] + m[9] * matrix.m[7] + m[10] * matrix.m[11] + m[11] * matrix.m[15];
		ret.m[12] = m[12] * matrix.m[0] + m[13] * matrix.m[4] + m[14] * matrix.m[8] + m[15] * matrix.m[12];
		ret.m[13] = m[12] * matrix.m[1] + m[13] * matrix.m[5] + m[14] * matrix.m[9] + m[15] * matrix.m[13];
		ret.m[14] = m[12] * matrix.m[2] + m[13] * matrix.m[6] + m[14] * matrix.m[10] + m[15] * matrix.m[14];
		ret.m[15] = m[12] * matrix.m[3] + m[13] * matrix.m[7] + m[14] * matrix.m[11] + m[15] * matrix.m[15];
		return ret;
	}

	Matrix operator*(const Matrix& matrix) { return mul(matrix); }

	Vec3 mulVec(const Vec3& v) {
		return Vec3(
			(v.x * m[0] + v.y * m[1] + v.z * m[2]),
			(v.x * m[4] + v.y * m[5] + v.z * m[6]),
			(v.x * m[8] + v.y * m[9] + v.z * m[10])
		);
	}

	Vec3 mulPoint(const Vec3& v) {
		Vec3 v1 = Vec3(
			(v.x * m[0] + v.y * m[1] + v.z * m[2]) + m[3],
			(v.x * m[4] + v.y * m[5] + v.z * m[6]) + m[7],
			(v.x * m[8] + v.y * m[9] + v.z * m[10]) + m[11]
		);
		return v1;
	}

	Vec3 mulPointAndPerspectiveDivide(const Vec3& v) {
		Vec3 v1 = Vec3(
			(v.x * m[0] + v.y * m[1] + v.z * m[2]) + m[3],
			(v.x * m[4] + v.y * m[5] + v.z * m[6]) + m[7],
			(v.x * m[8] + v.y * m[9] + v.z * m[10]) + m[11]
		);
		float w;
		w = (m[12] * v.x) + (m[13] * v.y) + (m[14] * v.z) + m[15];
		w = 1.f / w;
		return (v1 * w);
	}

	Matrix operator=(const Matrix& matrix) {
		memcpy(m, matrix.m, sizeof(float) * 16);
		return (*this);
	}

	// Unrolled inverse from MESA library
	Matrix invert() {
		Matrix inv;
		inv[0] = m[5] * m[10] * m[15] - m[5] * m[11] * m[14] - m[9] * m[6] * m[15] + m[9] * m[7] * m[14] + m[13] * m[6] * m[11] - m[13] * m[7] * m[10];
		inv[4] = -m[4] * m[10] * m[15] + m[4] * m[11] * m[14] + m[8] * m[6] * m[15] - m[8] * m[7] * m[14] - m[12] * m[6] * m[11] + m[12] * m[7] * m[10];
		inv[8] = m[4] * m[9] * m[15] - m[4] * m[11] * m[13] - m[8] * m[5] * m[15] + m[8] * m[7] * m[13] + m[12] * m[5] * m[11] - m[12] * m[7] * m[9];
		inv[12] = -m[4] * m[9] * m[14] + m[4] * m[10] * m[13] + m[8] * m[5] * m[14] - m[8] * m[6] * m[13] - m[12] * m[5] * m[10] + m[12] * m[6] * m[9];
		
		inv[1] = -m[1] * m[10] * m[15] + m[1] * m[11] * m[14] + m[9] * m[2] * m[15] - m[9] * m[3] * m[14] - m[13] * m[2] * m[11] + m[13] * m[3] * m[10];
		inv[5] = m[0] * m[10] * m[15] - m[0] * m[11] * m[14] - m[8] * m[2] * m[15] + m[8] * m[3] * m[14] + m[12] * m[2] * m[11] - m[12] * m[3] * m[10];
		inv[9] = -m[0] * m[9] * m[15] + m[0] * m[11] * m[13] + m[8] * m[1] * m[15] - m[8] * m[3] * m[13] - m[12] * m[1] * m[11] + m[12] * m[3] * m[9];
		inv[13] = m[0] * m[9] * m[14] - m[0] * m[10] * m[13] - m[8] * m[1] * m[14] + m[8] * m[2] * m[13] + m[12] * m[1] * m[10] - m[12] * m[2] * m[9];
		
		inv[2] = m[1] * m[6] * m[15] - m[1] * m[7] * m[14] - m[5] * m[2] * m[15] + m[5] * m[3] * m[14] + m[13] * m[2] * m[7] - m[13] * m[3] * m[6];
		inv[6] = -m[0] * m[6] * m[15] + m[0] * m[7] * m[14] + m[4] * m[2] * m[15] - m[4] * m[3] * m[14] - m[12] * m[2] * m[7] + m[12] * m[3] * m[6];
		inv[10] = m[0] * m[5] * m[15] - m[0] * m[7] * m[13] - m[4] * m[1] * m[15] + m[4] * m[3] * m[13] + m[12] * m[1] * m[7] - m[12] * m[3] * m[5];
		inv[14] = -m[0] * m[5] * m[14] + m[0] * m[6] * m[13] + m[4] * m[1] * m[14] - m[4] * m[2] * m[13] - m[12] * m[1] * m[6] + m[12] * m[2] * m[5];
		
		inv[3] = -m[1] * m[6] * m[11] + m[1] * m[7] * m[10] + m[5] * m[2] * m[11] - m[5] * m[3] * m[10] - m[9] * m[2] * m[7] + m[9] * m[3] * m[6];
		inv[7] = m[0] * m[6] * m[11] - m[0] * m[7] * m[10] - m[4] * m[2] * m[11] + m[4] * m[3] * m[10] + m[8] * m[2] * m[7] - m[8] * m[3] * m[6];
		inv[11] = -m[0] * m[5] * m[11] + m[0] * m[7] * m[9] + m[4] * m[1] * m[11] - m[4] * m[3] * m[9] - m[8] * m[1] * m[7] + m[8] * m[3] * m[5];
		inv[15] = m[0] * m[5] * m[10] - m[0] * m[6] * m[9] - m[4] * m[1] * m[10] + m[4] * m[2] * m[9] + m[8] * m[1] * m[6] - m[8] * m[2] * m[5];
		
		float det = m[0] * inv[0] + m[1] * inv[4] + m[2] * inv[8] + m[3] * inv[12];
		
		// This code should never be called. Add error handling if it is
		if (det == 0.f) {
			identity();
			det = 1.f;
		}
		det = 1.f / det;
		
		for (int i = 0; i < 16; i++) {
			inv[i] = inv[i] * det;
		}
		return inv;
	}

	static Matrix lookAt(const Vec3& from, const Vec3& to, const Vec3& up) {
		Matrix mat;
		Vec3 dir = (from - to).normalize();
		Vec3 left = up.cross(dir).normalize();
		Vec3 newUp = dir.cross(left);

		mat.a[0][0] = left.x;
		mat.a[0][1] = left.y;
		mat.a[0][2] = left.z;
		
		mat.a[1][0] = newUp.x;
		mat.a[1][1] = newUp.y;
		mat.a[1][2] = newUp.z;
		
		mat.a[2][0] = dir.x;
		mat.a[2][1] = dir.y;
		mat.a[2][2] = dir.z;
		
		mat.a[0][3] = -from.dot(left);
		mat.a[1][3] = -from.dot(newUp);
		mat.a[2][3] = -from.dot(dir);
		mat.a[3][3] = 1;
		return mat;
	}

	// FOV in degrees, outputs transposed Matrix for DX
	static Matrix perspective(const float n, const float f, float aspect, const float fov) {
		Matrix pers;
		memset(pers.m, 0, sizeof(float) * 16);
		float t = 1.0f / (tanf(fov * 0.5f * 3.141592654f / 180.f));

		pers.a[0][0] = t / aspect;
		pers.a[1][1] = t;
		pers.a[2][2] = -f / (f - n);
		pers.a[2][3] = -(f * n) / (f - n);
		pers.a[3][2] = -1.0f;
		return pers;
	}

	static Matrix rotateX(float theta) {
		Matrix mat;
		float ct = cosf(theta);
		float st = sinf(theta);

		mat.m[5] = ct;
		mat.m[6] = st;
		mat.m[9] = -st;
		mat.m[10] = ct;
		return mat;
	}

	static Matrix rotateY(float theta) {
		Matrix mat;
		float ct = cosf(theta);
		float st = sinf(theta);

		mat.m[0] = ct;
		mat.m[2] = -st;
		mat.m[8] = st;
		mat.m[10] = ct;
		return mat;
	}

	static Matrix rotateZ(float theta) {
		Matrix mat;
		float ct = cosf(theta);
		float st = sinf(theta);

		mat.m[0] = ct;
		mat.m[1] = st;
		mat.m[4] = -st;
		mat.m[5] = ct;
		return mat;
	}
};

class Frame {
public:
	Vec3 u;
	Vec3 v;
	Vec3 w;
	
	void fromVector(const Vec3& n) {
		// Gram-Schmit
		w = n.normalize();
		if (fabsf(w.x) > fabsf(w.y)) {
			float l = 1.f / sqrtf(w.x * w.x + w.z * w.z);
			u = Vec3(w.z * l, 0.f, -w.x * l);
		}

		else {
			float l = 1.f / sqrtf(w.y * w.y + w.z * w.z);
			u = Vec3(0.f, w.z * l, -w.y * l);
		}
		v = Cross(w, u);
	}

	void fromVectorTangent(const Vec3& n, const Vec3& t) {
		w = n.normalize();
		u = t.normalize();
		v = Cross(w, u);
	}

	Vec3 toLocal(const Vec3& vec) const { return Vec3(Dot(vec, u), Dot(vec, v), Dot(vec, w)); }
	Vec3 toWorld(const Vec3& vec) const { return ((u * vec.x) + (v * vec.y) + (w * vec.z)); }
};

class SphericalCoordinates {
public:
	static Vec3 sphericalToWorld(float theta, float phi) { return Vec3(cosf(phi) * sinf(theta), sinf(phi) * sinf(theta), cosf(theta)); }
	static float sphericalTheta(const Vec3& wi) { return acosf(wi.z); }
	static float sphericalPhi(const Vec3& wi) { float p = atan2f(wi.y, wi.x); return (p < 0.f) ? p + (2.f * M_PI) : p; }
};

template<typename T>
T& use() {
	static T t;
	return t;
}