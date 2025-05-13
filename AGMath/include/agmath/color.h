#pragma once

#include <cstdint>
#include <format>
#include <stdexcept>
#include <string>

#include "utilities.h"

namespace agm
{
	struct Color
	{
	public:

		float r;
		float g;
		float b;
		float a;

	public:

		static const Color BLACK;
		static const Color BLUE;
		static const Color CLEAR;
		static const Color CYAN;
		static const Color GRAY;
		static const Color GREEN;
		static const Color MAGENTA;
		static const Color RED;
		static const Color WHITE;
		static const Color YELLOW;

	public:

		constexpr Color()
			: r(0.f)
			, g(0.f)
			, b(0.f)
			, a(0.f)
		{
		}

		constexpr Color(float r, float g, float b)
			: r(r)
			, g(g)
			, b(b)
			, a(1.f)
		{
		}

		constexpr Color(float r, float g, float b, float a)
			: r(r)
			, g(g)
			, b(b)
			, a(a)
		{
		}

	public:

		constexpr float operator[](int32_t index) const
		{
			switch (index)
			{
			case 0: return r;
			case 1: return g;
			case 2: return b;
			case 3: return a;
			default: throw std::out_of_range("Invalid Color index!");
			}
		}

		constexpr float& operator[](int32_t index)
		{
			switch (index)
			{
			case 0: return r;
			case 1: return g;
			case 2: return b;
			case 3: return a;
			default: throw std::out_of_range("Invalid Color index!");
			}
		}

		constexpr Color operator*(float scalar) const
		{
			return Color(r * scalar, g * scalar, b * scalar, a * scalar);
		}

		constexpr Color operator/(float scalar) const
		{
			return Color(r / scalar, g / scalar, b / scalar, a / scalar);
		}

		constexpr Color operator+(const Color& other) const
		{
			return Color(r + other.r, g + other.g, b + other.b, a + other.a);
		}

		constexpr Color operator-(const Color& other) const
		{
			return Color(r - other.r, g - other.g, b - other.b, a - other.a);
		}

		constexpr Color operator*(const Color& other) const
		{
			return Color(r * other.r, g * other.g, b * other.b, a * other.a);
		}

		constexpr Color& operator*=(float scalar)
		{
			r *= scalar;
			g *= scalar;
			b *= scalar;
			a *= scalar;
			return *this;
		}

		constexpr Color& operator/=(float scalar)
		{
			r /= scalar;
			g /= scalar;
			b /= scalar;
			a /= scalar;
			return *this;
		}

		constexpr Color& operator+=(const Color& other)
		{
			r += other.r;
			g += other.g;
			b += other.b;
			a += other.a;
			return *this;
		}

		constexpr Color& operator-=(const Color& other)
		{
			r -= other.r;
			g -= other.g;
			b -= other.b;
			a -= other.a;
			return *this;
		}

		constexpr Color& operator*=(const Color& other)
		{
			r *= other.r;
			g *= other.g;
			b *= other.b;
			a *= other.a;
			return *this;
		}

		constexpr bool operator==(const Color& other) const
		{
			return r == other.r && g == other.g && b == other.b && a == other.a;
		}

		constexpr bool operator!=(const Color& other) const
		{
			return !(*this == other);
		}

		friend constexpr Color operator*(float scalar, const Color& c)
		{
			return Color(c.r * scalar, c.g * scalar, c.b * scalar, c.a * scalar);
		}

	public:

		Color Gamma() const
		{
			return Color(LinearToGammaSpace(r), LinearToGammaSpace(g), LinearToGammaSpace(b), a);
		}

		constexpr float GrayScale() const
		{
			return 0.299f * r + 0.587f * g + 0.114f * b;
		}

		Color Linear() const
		{
			return Color(GammaToLinearSpace(r), GammaToLinearSpace(g), GammaToLinearSpace(b), a);
		}

		constexpr float MaxColorComponent() const
		{
			return Max3(r, g, b);
		}

		constexpr bool Equals(const Color& other, float tolerance = LOOSE_EPSILON) const
		{
			return Abs(r - other.r) <= tolerance && Abs(g - other.g) <= tolerance && Abs(b - other.b) <= tolerance && Abs(a - other.a) <= tolerance;
		}

		std::string ToString() const
		{
			return std::format("({:.3f}, {:.3f}, {:.3f}, {:.3f})", r, g, b, a);
		}

	public:

		static Color HSVToRGB(float H, float S, float V, bool hdr = true)
		{
			Color result = WHITE;

			if (S == 0.f)
			{
				result.r = V;
				result.g = V;
				result.b = V;
			}
			else if (V == 0.f)
			{
				result.r = 0.f;
				result.g = 0.f;
				result.b = 0.f;
			}
			else
			{
				result.r = 0.f;
				result.g = 0.f;
				result.b = 0.f;

				float scaledHue = H * 6.f;
				int32_t sector = FloorToInt(scaledHue);
				float fractional = scaledHue - (float)sector;

				float p = V * (1.f - S);
				float q = V * (1.f - S * fractional);
				float t = V * (1.f - S * (1.f - fractional));

				switch (sector)
				{
				case 0:
					result.r = V;
					result.g = t;
					result.b = p;
					break;
				case 1:
					result.r = q;
					result.g = V;
					result.b = p;
					break;
				case 2:
					result.r = p;
					result.g = V;
					result.b = t;
					break;
				case 3:
					result.r = p;
					result.g = q;
					result.b = V;
					break;
				case 4:
					result.r = t;
					result.g = p;
					result.b = V;
					break;
				case 5:
					result.r = V;
					result.g = p;
					result.b = q;
					break;
				case 6:
					result.r = V;
					result.g = t;
					result.b = p;
					break;
				case -1:
					result.r = V;
					result.g = p;
					result.b = q;
					break;
				}

				if (!hdr)
				{
					result.r = Clamp(result.r, 0.f, 1.f);
					result.g = Clamp(result.g, 0.f, 1.f);
					result.b = Clamp(result.b, 0.f, 1.f);
				}
			}

			return result;
		}

		static constexpr Color Lerp(const Color& a, const Color& b, float t)
		{
			t = Clamp01(t);
			return Color(a.r + (b.r - a.r) * t, a.g + (b.g - a.g) * t, a.b + (b.b - a.b) * t, a.a + (b.a - a.a) * t);
		}

		static constexpr Color LerpUnclamped(const Color& a, const Color& b, float t)
		{
			return Color(a.r + (b.r - a.r) * t, a.g + (b.g - a.g) * t, a.b + (b.b - a.b) * t, a.a + (b.a - a.a) * t);
		}

		static constexpr void RGBToHSV(Color rgbColor, float& outH, float& outS, float& outV)
		{
			if (rgbColor.b > rgbColor.g && rgbColor.b > rgbColor.r)
			{
				RGBToHSVHelper(4.f, rgbColor.b, rgbColor.r, rgbColor.g, outH, outS, outV);
			}
			else if (rgbColor.g > rgbColor.r)
			{
				RGBToHSVHelper(2.f, rgbColor.g, rgbColor.b, rgbColor.r, outH, outS, outV);
			}
			else
			{
				RGBToHSVHelper(0.f, rgbColor.r, rgbColor.g, rgbColor.b, outH, outS, outV);
			}
		}

	private:

		static constexpr void RGBToHSVHelper(float offset, float dominantcolor, float color1, float color2, float& outH, float& outS, float& outV)
		{
			outV = dominantcolor;
			if (outV != 0.f)
			{
				float minColor = (color1 < color2) ? color1 : color2;
				float chroma = outV - minColor;
				if (chroma != 0.f)
				{
					outS = chroma / outV;
					outH = offset + (color1 - color2) / chroma;
				}
				else
				{
					outS = 0.f;
					outH = offset + (color1 - color2);
				}

				outH /= 6.f;
				if (outH < 0.f)
				{
					outH += 1.f;
				}
			}
			else
			{
				outS = 0.f;
				outH = 0.f;
			}
		}
	};

	inline const Color Color::BLACK = Color(0.f, 0.f, 0.f, 1.f);
	inline const Color Color::BLUE = Color(0.f, 0.f, 1.f, 1.f);
	inline const Color Color::CLEAR = Color(0.f, 0.f, 0.f, 0.f);
	inline const Color Color::CYAN = Color(0.f, 1.f, 1.f, 1.f);
	inline const Color Color::GRAY = Color(0.5f, 0.5f, 0.5f, 1.f);
	inline const Color Color::GREEN = Color(0.f, 1.f, 0.f, 1.f);
	inline const Color Color::MAGENTA = Color(1.f, 0.f, 1.f, 1.f);
	inline const Color Color::RED = Color(1.f, 0.f, 0.f, 1.f);
	inline const Color Color::WHITE = Color(1.f, 1.f, 1.f, 1.f);
	inline const Color Color::YELLOW = Color(1.f, 0.92f, 0.016f, 1.f);
}

