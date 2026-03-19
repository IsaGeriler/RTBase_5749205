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

class BoxFilter : public ImageFilter
{
public:
	float filter(float x, float y) const
	{
		if (fabsf(x) < 0.5f && fabs(y) < 0.5f)
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

class GaussianFilter : public ImageFilter {
public:
	// TO:DO
};

class MitchellNetravali : public ImageFilter {
public:
	// TO:DO
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

		// Gamma (change between 1.4, 1.8, 2.2, 2.6)
		const float gamma = 2.2f;

		// Get output components
		float L_out_r = use_gamma ? std::powf((film[index].r / max.r), (1.f / gamma)) : (film[index].r / max.r);
		float L_out_g = use_gamma ? std::powf((film[index].g / max.g), (1.f / gamma)) : (film[index].g / max.g);
		float L_out_b = use_gamma ? std::powf((film[index].b / max.b), (1.f / gamma)) : (film[index].b / max.b);
		
		film[index].r = L_out_r;
		film[index].g = L_out_g;
		film[index].b = L_out_b;

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

		// Gamma (change between 1.4, 1.8, 2.2, 2.6)
		const float gamma = 2.2f;

		// Get output components
		float L_out_r = std::powf((film[index].r * std::powf(2, exposure)), (1.f / gamma));
		float L_out_g = std::powf((film[index].g * std::powf(2, exposure)), (1.f / gamma));
		float L_out_b = std::powf((film[index].b * std::powf(2, exposure)), (1.f / gamma));

		film[index].r = L_out_r;
		film[index].g = L_out_g;
		film[index].b = L_out_b;

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

		// Gamma (change between 1.4, 1.8, 2.2, 2.6)
		const float gamma = 2.2f;

		// Get output components
		// Lout = (Lin / 1 + Lin)^(1 / gamma)
		// This is effectively a sigmoid function with a gamma correction
		float L_out_r = std::powf(film[index].r / (1.f + film[index].r), (1.f / gamma));
		float L_out_g = std::powf(film[index].g / (1.f + film[index].g), (1.f / gamma));
		float L_out_b = std::powf(film[index].b / (1.f + film[index].b), (1.f / gamma));

		film[index].r = L_out_r;
		film[index].g = L_out_g;
		film[index].b = L_out_b;

		r = (unsigned char)(film[index].r * 255.f);
		g = (unsigned char)(film[index].g * 255.f);
		b = (unsigned char)(film[index].b * 255.f);
	}
	// Tonemap: Filmic (Uncharted 2, John Hable)
	void tonemap_uncharted_hable(int x, int y, unsigned char& r, unsigned char& g, unsigned char& b)
	{
		// Input colour
		unsigned int index = (y * width) + x;
		film[index].r = (float)(r / 255.f);
		film[index].g = (float)(g / 255.f);
		film[index].b = (float)(b / 255.f);

		// Lout = [C(Lin) / C(W)]^(1 / gamma)
		// where, C(x) = ((x (Ax + CB) + DE) / (x (Ax + B) + DF)) - (E/F) -- x is an arbitrary variable
		// A = 0.15, B = 0.5, C = 0.1, D = 0.2, E = 0.02, F = 0.3, W = 11.2

		// Define the variables 
		const float A = 0.15f, B = 0.5f, C = 0.1f, D = 0.2f, E = 0.02f, F = 0.3f, W = 11.2f;

		// Gamma (change between 1.4, 1.8, 2.2, 2.6)
		const float gamma = 2.2f;

		// Formulate Filmic Tone Mapping (Uncharted 2, Hable)
		float c_Lin_r = ((film[index].r * (A * film[index].r + C * B) + D * E) / (film[index].r * (A * film[index].r + B) + D * F)) - (E / F);
		float c_Lin_g = ((film[index].g * (A * film[index].g + C * B) + D * E) / (film[index].g * (A * film[index].g + B) + D * F)) - (E / F);
		float c_Lin_b = ((film[index].b * (A * film[index].b + C * B) + D * E) / (film[index].b * (A * film[index].b + B) + D * F)) - (E / F);
		float c_W = ((W * (A * W + C * B) + D * E) / (W * (A * W + B) + D * F)) - (E / F);

		// Set Gamma corrected output colour to the film
		float L_out_r = std::powf((c_Lin_r / c_W), (1.f / gamma));
		float L_out_g = std::powf((c_Lin_g / c_W), (1.f / gamma));
		float L_out_b = std::powf((c_Lin_b / c_W), (1.f / gamma));

		film[index].r = L_out_r;
		film[index].g = L_out_g;
		film[index].b = L_out_b;

		r = (unsigned char)(film[index].r * 255.f);
		g = (unsigned char)(film[index].g * 255.f);
		b = (unsigned char)(film[index].b * 255.f);
	}
	// Tonemap: ACES Filmic Curve
	// Adapted from: https://knarkowicz.wordpress.com/2016/01/06/aces-filmic-tone-mapping-curve/
	void tonemap_aces_filmic_curve(int x, int y, unsigned char& r, unsigned char& g, unsigned char& b)
	{
		// Input colour
		unsigned int index = (y * width) + x;
		film[index].r = (float)(r / 255.f);
		film[index].g = (float)(g / 255.f);
		film[index].b = (float)(b / 255.f);

		// Define the variables 
		const float A = 2.51f, B = 0.03f, C = 2.43f, D = 0.59f, E = 0.14f;

		// Get output components (No need for gamma correction for this)
		// Lout = saturate((x * (a * x + b)) / (x * (c * x + d) + e))
		float L_out_r = clamp((film[index].r * (A * film[index].r + B) / (film[index].r * (C * film[index].r + E))), 0.f, 255.f);
		float L_out_g = clamp((film[index].g * (A * film[index].g + B) / (film[index].g * (C * film[index].g + E))), 0.f, 255.f);
		float L_out_b = clamp((film[index].b * (A * film[index].b + B) / (film[index].b * (C * film[index].b + E))), 0.f, 255.f);

		film[index].r = L_out_r;
		film[index].g = L_out_g;
		film[index].b = L_out_b;

		r = (unsigned char)(film[index].r * 255.f);
		g = (unsigned char)(film[index].g * 255.f);
		b = (unsigned char)(film[index].b * 255.f);
	}
	// Tonemap: Jim Hejl and Richard Burgess-Dawson
	// Adapted from: http://filmicworlds.com/blog/filmic-tonemapping-operators/
	void tonemap_jim_hejl_richard_burgess_dawson(int x, int y, unsigned char& r, unsigned char& g, unsigned char& b)
	{
		// Input colour
		unsigned int index = (y * width) + x;
		film[index].r = (float)(r / 255.f);
		film[index].g = (float)(g / 255.f);
		film[index].b = (float)(b / 255.f);

		// Get output components (No need for gamma correction for this)
		// Lout = (C(x) * (6.2 * C(x) + 0.5)) / (C(x) * (6.2 * C(x) + 1.7) + 0.06)
		// where, C(x) = max(0, Lin - 0.004)
		float c_r = std::fmaxf(0.f, film[index].r - 0.004f);
		float c_g = std::fmaxf(0.f, film[index].g - 0.004f);
		float c_b = std::fmaxf(0.f, film[index].b - 0.004f);

		float L_out_r = (c_r * (6.2f * c_r + 0.5f)) / (c_r * (6.2f * c_r + 1.7f) + 0.06f);
		float L_out_g = (c_g * (6.2f * c_g + 0.5f)) / (c_g * (6.2f * c_g + 1.7f) + 0.06f);
		float L_out_b = (c_b * (6.2f * c_b + 0.5f)) / (c_b * (6.2f * c_b + 1.7f) + 0.06f);

		film[index].r = L_out_r;
		film[index].g = L_out_g;
		film[index].b = L_out_b;

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
			case 5: { tonemap_uncharted_hable(x, y, r, g, b); break; }					// Filmic (Uncharted, Hable)
			case 6: { tonemap_aces_filmic_curve(x, y, r, g, b); break; }				// ACES Filmic Curve
			case 7: { tonemap_jim_hejl_richard_burgess_dawson(x, y, r, g, b); break; }  // Jim Hejl and Richard Burgess-Dawson
			default: tonemap_linear(x, y, r, g, b, true);								// Default Tonemap Operator
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