#pragma once

#include <cstdint>
#include <format>
#include <stdexcept>
#include <string>

#include "utilities.h"

namespace agm
{
	struct Color32
	{
	public:

		uint8_t r;
		uint8_t g;
		uint8_t b;
		uint8_t a;

	public:

		constexpr Color32()
			: r(0)
			, g(0)
			, b(0)
			, a(0)
		{
		}

		constexpr Color32(uint8_t r, uint8_t g, uint8_t b, uint8_t a)
			: r(r)
			, g(g)
			, b(b)
			, a(a)
		{
		}

	public:

		constexpr uint8_t operator[](int32_t index) const
		{
			switch (index)
			{
			case 0: return r;
			case 1: return g;
			case 2: return b;
			case 3: return a;
			default: throw std::out_of_range("Invalid Color32 index!");
			}
		}

		constexpr uint8_t& operator[](int32_t index)
		{
			switch (index)
			{
			case 0: return r;
			case 1: return g;
			case 2: return b;
			case 3: return a;
			default: throw std::out_of_range("Invalid Color32 index!");
			}
		}

		constexpr bool operator==(const Color32& other) const
		{
			return r == other.r && g == other.g && b == other.b && a == other.a;
		}

		constexpr bool operator!=(const Color32& other) const
		{
			return !(*this == other);
		}

	public:

		static constexpr Color32 Lerp(const Color32& a, const Color32& b, float t)
		{
			t = Clamp01(t);
			return Color32(
				(uint8_t)((float)(int)a.r + (float)(b.r - a.r) * t),
				(uint8_t)((float)(int)a.g + (float)(b.g - a.g) * t),
				(uint8_t)((float)(int)a.b + (float)(b.b - a.b) * t),
				(uint8_t)((float)(int)a.a + (float)(b.a - a.a) * t)
			);
		}

		static constexpr Color32 LerpUnclamped(const Color32& a, const Color32& b, float t)
		{
			return Color32(
				(uint8_t)((float)(int)a.r + (float)(b.r - a.r) * t),
				(uint8_t)((float)(int)a.g + (float)(b.g - a.g) * t),
				(uint8_t)((float)(int)a.b + (float)(b.b - a.b) * t),
				(uint8_t)((float)(int)a.a + (float)(b.a - a.a) * t)
			);
		}

		std::string ToString() const
		{
			return std::format("({}, {}, {}, {})", r, g, b, a);
		}
	};
}

