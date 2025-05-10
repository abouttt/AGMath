#pragma once

#include <array>
#include <cstddef>
#include <format>
#include <string>

#include "utilities.h"

namespace agm
{
	struct Color32
	{
		union
		{
			struct
			{
				uint8_t r, g, b, a;
			};

			std::array<uint8_t, 4> data;
		};

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

		constexpr uint8_t operator[](size_t index) const
		{
			return data[index];
		}

		constexpr uint8_t& operator[](size_t index)
		{
			return data[index];
		}

		constexpr bool operator==(const Color32& other) const
		{
			return r == other.r && g == other.g && b == other.b && a == other.a;
		}

		constexpr bool operator!=(const Color32& other) const
		{
			return !(*this == other);
		}

		static inline constexpr Color32 Lerp(const Color32& a, const Color32& b, float t)
		{
			t = Clamp01(t);
			return Color32(
				(uint8_t)((float)(int)a.r + (float)(b.r - a.r) * t),
				(uint8_t)((float)(int)a.g + (float)(b.g - a.g) * t),
				(uint8_t)((float)(int)a.b + (float)(b.b - a.b) * t),
				(uint8_t)((float)(int)a.a + (float)(b.a - a.a) * t)
			);
		}

		static inline constexpr Color32 LerpUnclamped(const Color32& a, const Color32& b, float t)
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

