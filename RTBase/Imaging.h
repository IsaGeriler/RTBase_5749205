#pragma once

#include "Core.h"
#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"
#define STB_IMAGE_WRITE_IMPLEMENTATION
#define __STDC_LIB_EXT1__
#include "stb_image_write.h"

// Stop warnings about buffer overruns if size is zero. Size should never be zero and if it is the code handles it.
#pragma warning( disable : 6386)

constexpr float texelScale = 1.0f / 255.0f;

class Texture
{
public:
	Colour* texels;
	float* alpha;
	int width;
	int height;
	int channels;
	void loadDefault()
	{
		width = 1;
		height = 1;
		channels = 3;
		texels = new Colour[1];
		texels[0] = Colour(1.0f, 1.0f, 1.0f);
	}
	void load(std::string filename)
	{
		alpha = NULL;
		if (filename.find(".hdr") != std::string::npos)
		{
			float* textureData = stbi_loadf(filename.c_str(), &width, &height, &channels, 0);
			if (width == 0 || height == 0)
			{
				loadDefault();
				return;
			}
			texels = new Colour[width * height];
			for (int i = 0; i < (width * height); i++)
			{
				texels[i] = Colour(textureData[i * channels], textureData[(i * channels) + 1], textureData[(i * channels) + 2]);
			}
			stbi_image_free(textureData);
			return;
		}
		unsigned char* textureData = stbi_load(filename.c_str(), &width, &height, &channels, 0);
		if (width == 0 || height == 0)
		{
			loadDefault();
			return;
		}
		texels = new Colour[width * height];
		for (int i = 0; i < (width * height); i++)
		{
			texels[i] = Colour(textureData[i * channels] / 255.0f, textureData[(i * channels) + 1] / 255.0f, textureData[(i * channels) + 2] / 255.0f);
		}
		if (channels == 4)
		{
			alpha = new float[width * height];
			for (int i = 0; i < (width * height); i++)
			{
				alpha[i] = textureData[(i * channels) + 3] / 255.0f;
			}
		}
		stbi_image_free(textureData);
	}
	Colour sample(const float tu, const float tv) const
	{
		Colour tex;
		float u = std::max(0.0f, fabsf(tu)) * width;
		float v = std::max(0.0f, fabsf(tv)) * height;
		int x = (int)floorf(u);
		int y = (int)floorf(v);
		float frac_u = u - x;
		float frac_v = v - y;
		float w0 = (1.0f - frac_u) * (1.0f - frac_v);
		float w1 = frac_u * (1.0f - frac_v);
		float w2 = (1.0f - frac_u) * frac_v;
		float w3 = frac_u * frac_v;
		x = x % width;
		y = y % height;
		Colour s[4];
		s[0] = texels[y * width + x];
		s[1] = texels[y * width + ((x + 1) % width)];
		s[2] = texels[((y + 1) % height) * width + x];
		s[3] = texels[((y + 1) % height) * width + ((x + 1) % width)];
		tex = (s[0] * w0) + (s[1] * w1) + (s[2] * w2) + (s[3] * w3);
		return tex;
	}
	float sampleAlpha(const float tu, const float tv) const
	{
		if (alpha == NULL)
		{
			return 1.0f;
		}
		float tex;
		float u = std::max(0.0f, fabsf(tu)) * width;
		float v = std::max(0.0f, fabsf(tv)) * height;
		int x = (int)floorf(u);
		int y = (int)floorf(v);
		float frac_u = u - x;
		float frac_v = v - y;
		float w0 = (1.0f - frac_u) * (1.0f - frac_v);
		float w1 = frac_u * (1.0f - frac_v);
		float w2 = (1.0f - frac_u) * frac_v;
		float w3 = frac_u * frac_v;
		x = x % width;
		y = y % height;
		float s[4];
		s[0] = alpha[y * width + x];
		s[1] = alpha[y * width + ((x + 1) % width)];
		s[2] = alpha[((y + 1) % height) * width + x];
		s[3] = alpha[((y + 1) % height) * width + ((x + 1) % width)];
		tex = (s[0] * w0) + (s[1] * w1) + (s[2] * w2) + (s[3] * w3);
		return tex;
	}
	~Texture()
	{
		delete[] texels;
		if (alpha != NULL)
		{
			delete alpha;
		}
	}
};

class ImageFilter
{
public:
	virtual float filter(const float x, const float y) const = 0;
	virtual int size() const = 0;
};
// Box Filter
class BoxFilter : public ImageFilter
{
public:
	float filter(float x, float y) const
	{
		if (fabsf(x) <= 0.5f && fabs(y) <= 0.5f)
		{
			return 1.0f;
		}
		return 0;
	}
	int size() const
	{
		return 0;
	}
};
// Triangle Filter
// Adapted from: https://www.pbr-book.org/4ed/Sampling_and_Reconstruction/Image_Reconstruction#fragment-FilterInterface-1
class TriangleFilter : public ImageFilter {
public:
	// TO:DO
	float filter(float x, float y) const
	{
		return std::fmaxf(0, 1.f - std::fabsf(x)) * std::fmaxf(0, 1.f - std::fabsf(y));
	}
	int size() const
	{
		return 0;
	}
};
// Gaussian Filter
inline float Gaussian(float d, float radius, float alpha)
{
	// alpha: smoothness, related with the filter size passed as parameter
	return std::powf(2.7182817f, (-1 * alpha * d * d)) - std::powf(2.7182817f, (- 1 * alpha * radius * radius));
}
class GaussianFilter : public ImageFilter
{
private:
	float radius, alpha;
public:
	GaussianFilter(float _radius, float _alpha) : radius(_radius), alpha(_alpha) {}
	float filter(float x, float y) const { return Gaussian(x, radius, alpha) * Gaussian(y, radius, alpha); }
	int size() const { return 0; }
};
// Mitchell-Netravali Filter
inline float MitchellNetravali(float d, float B, float C)
{
	// Separable equation for Mitchell-Netravali Filter
	if (d >= 0 && d < 1)
	{
		return (1.f / 6.f) * ((12 - 9 * B - 6 * C) * std::powf(std::fabsf(d), 3) + (-18 + 12 * B + 6 * C) * std::powf(std::fabsf(d), 2) + (6 - 2 * B));
	}
	else if (d >= 1 && d < 2)
	{
		return (1.f / 6.f) * ((-B - 6 * C) * std::powf(std::fabsf(d), 3) + (6 * B + 30 * C) * std::powf(std::fabsf(d), 2) + (-12 * B - 48 * C) * std::fabsf(d) + (8 * B + 24 * C));
	}
	else if (d >= 2)
	{
		return 0;
	}
}
class MitchellNetravaliFilter : public ImageFilter
{
private:
	 const float radius = 2.f;  // Radius is fixed at 2 for Mitchell-Netravali Filter
	 float B = 1.f / 3.f;
	 float C = 1.f / 3.f;
public:
	float filter(float x, float y) const
	{
		return MitchellNetravali(2.f * x / radius, B, C) * MitchellNetravali(2.f * y / radius, B, C);
	}
	int size() const
	{
		return 0;
	}
};
// Windowed Sinc Filter (Lanczos Sinc Filter)
// Adapted from: https://www.pbr-book.org/4ed/Sampling_and_Reconstruction/Image_Reconstruction#fragment-FilterInterface-1
inline float windowedSinc(float d, float radius, float tau)
{
	// sinc(x) = sin(x) / x
	if (std::fabsf(d) > radius) return 0;
	return (sinf(M_PI * d) / (M_PI * d)) * (sinf(M_PI * d / tau) / (M_PI * d / tau));
}

class LanczosSincFilter : public ImageFilter {
private:
	float radius, tau;
public:
	LanczosSincFilter(float _radius, float _tau) : radius(_radius), tau(_tau) {}
	float filter(float x, float y) const
	{
		return windowedSinc(x, radius, tau) * windowedSinc(y, radius, tau);
	}
	int size() const
	{
		return 0;
	}
};

class Film
{
public:
	Colour* film;
	unsigned int width;
	unsigned int height;
	int SPP;
	ImageFilter* filter;
	void splat(const float x, const float y, const Colour& L)
	{
		// Code to splat a smaple with colour L into the image plane using an ImageFilter
		float filterWeights[25];   // Storage to cache weights
		unsigned int indices[25];  // Store indices to minimize computations
		unsigned int used = 0;
		float total = 0;
		int size = filter->size();
		for (int i = -size; i <= size; i++) {
			for (int j = -size; j <= size; j++) {
				int px = (int)x + j;
				int py = (int)y + i;
				if (px >= 0 && px < width && py >= 0 && py < height) {
					indices[used] = (py * width) + px;
					filterWeights[used] = filter->filter(px - x, py - y);
					total += filterWeights[used];
					used++;
				}
			}
		}
		for (int i = 0; i < used; i++) {
			film[indices[i]] = film[indices[i]] + (L * filterWeights[i] / total);
		}
	}
	// Implement different tone mapping algorithms for comparisons
	// Tonemap: Linear Tonemap with/without gamma
	void tonemap_linear(int x, int y, unsigned char& r, unsigned char& g, unsigned char& b, bool use_gamma = false)
	{
		// Input colour
		unsigned int index = (y * width) + x;
		film[index].r = (float)(r / 255.f);
		film[index].g = (float)(g / 255.f);
		film[index].b = (float)(b / 255.f);

		// Max colour
		Colour max(255.f, 255.f, 255.f);

		// Gamma (change between 1.4, 1.8, 2.2, 2.4, 2.6)
		float gamma = 2.2f;

		// Obtain Luminance Values
		const float Lout = use_gamma ? std::powf((film[index].Lum() / max.Lum()), (1.f / gamma)) : (film[index].Lum() / max.Lum());
		const float Ldisp = clamp(Lout, 0.f, 255.f);
		
		// Obtain the output colour
		film[index] = film[index] * Ldisp;

		// Apply Gamma Correction for Monitor Display
		film[index].applyGammaCorrection(gamma);

		r = (unsigned char)(film[index].r * 255.f);
		g = (unsigned char)(film[index].g * 255.f);
		b = (unsigned char)(film[index].b * 255.f);
	}
	// Tonemap: Linear Tonemap with Gamma and Exposure
	void tonemap_linear_gamma_exposure(int x, int y, unsigned char& r, unsigned char& g, unsigned char& b, float exposure = 1.f)
	{
		// Input colour
		unsigned int index = (y * width) + x;
		film[index].r = (float)(r / 255.f);
		film[index].g = (float)(g / 255.f);
		film[index].b = (float)(b / 255.f);

		// Gamma (change between 1.4, 1.8, 2.2, 2.4, 2.6)
		float gamma = 2.2f;

		// Obtain Luminance Values
		const float Lout = std::powf(film[index].Lum() * std::powf(2, exposure), (1.f / gamma));
		const float Ldisp = clamp(Lout, 0.f, 255.f);

		// Obtain the output colour
		film[index] = film[index] * Ldisp;

		// Apply Gamma Correction for Monitor Display
		film[index].applyGammaCorrection(gamma);

		r = (unsigned char)(film[index].r * 255.f);
		g = (unsigned char)(film[index].g * 255.f);
		b = (unsigned char)(film[index].b * 255.f);
	}
	// Tonemap: Reinhard Global
	void tonemap_reinhard_global(int x, int y, unsigned char& r, unsigned char& g, unsigned char& b)
	{
		unsigned int index = (y * width) + x;
		film[index].r = (float)(r / 255.f);
		film[index].g = (float)(g / 255.f);
		film[index].b = (float)(b / 255.f);

		// Gamma (change between 1.4, 1.8, 2.2, 2.4, 2.6)
		float gamma = 2.2f;

		// Obtain Luminance Values
		// Lout = (Lin / 1 + Lin)^(1 / gamma)
		// This is effectively a sigmoid function with a gamma correction
		const float Lout = std::powf((film[index].Lum() / (1.f + film[index].Lum())), (1.f / gamma));
		const float Ldisp = clamp(Lout, 0.f, 255.f);

		// Obtain the output colour
		film[index] = film[index] * Ldisp;

		// Apply Gamma Correction for Monitor Display
		film[index].applyGammaCorrection(gamma);

		r = (unsigned char)(film[index].r * 255.f);
		g = (unsigned char)(film[index].g * 255.f);
		b = (unsigned char)(film[index].b * 255.f);
	}
	// Tonemap: Filmic (Uncharted 2, John Hable)
	inline float uncharted_hable_formula(float x)
	{
		// A = 0.15, B = 0.5, C = 0.1, D = 0.2, E = 0.02, F = 0.3
		// C(x) = ((x (Ax + CB) + DE) / (x (Ax + B) + DF)) - (E/F)
		return ((x * (0.15f * x + 0.1f * 0.5f) + 0.2f * 0.02f) / (x * (0.15 * x + 0.5f) + 0.2f * 0.3f)) - (0.02f / 0.3f);
	}
	void tonemap_uncharted_hable(int x, int y, unsigned char& r, unsigned char& g, unsigned char& b)
	{
		// Input colour
		unsigned int index = (y * width) + x;
		film[index].r = (float)(r / 255.f);
		film[index].g = (float)(g / 255.f);
		film[index].b = (float)(b / 255.f);

		// Gamma (change between 1.4, 1.8, 2.2, 2.4, 2.6)
		float gamma = 2.2f;

		// Obtain Luminance Values
		// Lout = [C(Lin) / C(W)]^(1 / gamma)
		const float CLin = uncharted_hable_formula(film[index].Lum());
		const float CW = uncharted_hable_formula(11.2f);
		const float Lout = std::powf((CLin / CW), (1.f / gamma));
		const float Ldisp = clamp(Lout, 0.f, 255.f);

		// Obtain the output colour
		film[index] = film[index] * Ldisp;

		// Apply Gamma Correction for Monitor Display
		film[index].applyGammaCorrection(gamma);

		r = (unsigned char)(film[index].r * 255.f);
		g = (unsigned char)(film[index].g * 255.f);
		b = (unsigned char)(film[index].b * 255.f);
	}
	// Tonemap: ACES Filmic Curve
	// Adapted from: https://knarkowicz.wordpress.com/2016/01/06/aces-filmic-tone-mapping-curve/
	inline float aces_filmic_curve_formula(float x)
	{
		// A = 2.51, B = 0.03, C = 2.43, D = 0.59, E = 0.14
		// Lout = saturate((x * 0.6f * (a * x * 0.6f + b)) / (x * 0.6f * (c * x * 0.6f + d) + e))
		// Note: Multiplied the input components with 0.6f for the original ACES Filmic Curve
		return clamp((x * 0.6f * (2.51f * x * 0.6f + 0.03f) / (x * 0.6f * (2.43f * x * 0.6f + 0.14f))), 0.f, 255.f);
	}
	void tonemap_aces_filmic_curve(int x, int y, unsigned char& r, unsigned char& g, unsigned char& b)
	{
		// Input colour
		unsigned int index = (y * width) + x;
		film[index].r = (float)(r / 255.f);
		film[index].g = (float)(g / 255.f);
		film[index].b = (float)(b / 255.f);

		// Define the variables 
		const float A = 2.51f, B = 0.03f, C = 2.43f, D = 0.59f, E = 0.14f;

		// Gamma - 2.4 works best for ACES Filmic Curve (but can change between 1.4, 1.8, 2.2, 2.4, 2.6)
		float gamma = 2.4f;

		// Obtain Luminance Values
		float Lout = std::powf(aces_filmic_curve_formula(film[index].Lum()), (1.f / gamma));
		float Ldisp = clamp(Lout, 0.f, 255.f);

		// Obtain the output colour
		film[index] = film[index] * Ldisp;

		// Apply Gamma Correction for Monitor Display
		film[index].applyGammaCorrection(gamma);

		r = (unsigned char)(film[index].r * 255.f);
		g = (unsigned char)(film[index].g * 255.f);
		b = (unsigned char)(film[index].b * 255.f);
	}
	// Tonemap: Jim Hejl and Richard Burgess-Dawson
	// Adapted from: http://filmicworlds.com/blog/filmic-tonemapping-operators/
	inline float jim_hejl_richard_burgess_dawson_formula(float x)
	{
		return (x * (6.2f * x + 0.5f)) / (x * (6.2f * x + 1.7f) + 0.06f);
	}
	void tonemap_jim_hejl_richard_burgess_dawson(int x, int y, unsigned char& r, unsigned char& g, unsigned char& b)
	{
		// Input colour
		unsigned int index = (y * width) + x;
		film[index].r = (float)(r / 255.f);
		film[index].g = (float)(g / 255.f);
		film[index].b = (float)(b / 255.f);

		// Gamma (change between 1.4, 1.8, 2.2, 2.4, 2.6)
		float gamma = 2.2f;

		// Obtain Luminance Values - No need for gamma correction for this
		// Lout = (C(x) * (6.2 * C(x) + 0.5)) / (C(x) * (6.2 * C(x) + 1.7) + 0.06)
		// C(x) = max(0, Lin - 0.004)
		float Lout = jim_hejl_richard_burgess_dawson_formula(std::fmaxf(0.f, film[index].Lum() - 0.004));
		float Ldisp = clamp(Lout, 0.f, 255.f);

		// Obtain the output colour
		film[index] = film[index] * Ldisp;

		// Apply Gamma Correction for Monitor Display
		film[index].applyGammaCorrection(gamma);

		r = (unsigned char)(film[index].r * 255.f);
		g = (unsigned char)(film[index].g * 255.f);
		b = (unsigned char)(film[index].b * 255.f);
	}
	void tonemap(int x, int y, unsigned char& r, unsigned char& g, unsigned char& b, int tonemap_operator)
	{
		// Return a tonemapped pixel at coordinates x, y
		switch (tonemap_operator)
		{
			case 1: { tonemap_linear(x, y, r, g, b); break; }							// Linear without gamma
			case 2: { tonemap_linear(x, y, r, g, b, true); break; }						// Linear with gamma
			case 3: { tonemap_linear_gamma_exposure(x, y, r, g, b, 0.f); break; }		// Linear with gamma and exposure
			case 4: { tonemap_reinhard_global(x, y, r, g, b); break; }					// Reinhard Global
			case 5: { tonemap_uncharted_hable(x, y, r, g, b); break; }					// Filmic (Uncharted 2, Hable)
			case 6: { tonemap_aces_filmic_curve(x, y, r, g, b); break; }				// ACES Filmic Curve
			case 7: { tonemap_jim_hejl_richard_burgess_dawson(x, y, r, g, b); break; }  // Jim Hejl and Richard Burgess-Dawson
			default: tonemap_reinhard_global(x, y, r, g, b);							// Default Tonemap Operator
		}
	}
	// Do not change any code below this line
	void init(int _width, int _height, ImageFilter* _filter)
	{
		width = _width;
		height = _height;
		film = new Colour[width * height];
		clear();
		filter = _filter;
	}
	void clear()
	{
		memset(film, 0, width * height * sizeof(Colour));
		SPP = 0;
	}
	void incrementSPP()
	{
		SPP++;
	}
	void save(std::string filename)
	{
		Colour* hdrpixels = new Colour[width * height];
		for (unsigned int i = 0; i < (width * height); i++)
		{
			hdrpixels[i] = film[i] / (float)SPP;
		}
		stbi_write_hdr(filename.c_str(), width, height, 3, (float*)hdrpixels);
		delete[] hdrpixels;
	}
};