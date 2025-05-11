#pragma once

#include <cmath>
#include <cstdint>

#include "color.h"
#include "color32.h"
#include "utilities.h"
#include "vector2.h"
#include "vector2_int.h"
#include "vector3.h"
#include "vector3_int.h"
#include "vector4.h"

namespace agm
{
	inline constexpr Color ToColor(const Color32& c)
	{
		return Color(float(c.r) / 255.f, float(c.g) / 255.f, float(c.b) / 255.f, float(c.a) / 255.f);
	}

	inline Color32 ToColor32(const Color& c)
	{
		return Color32(
			(uint8_t)std::round(Clamp01(c.r) * 255.f),
			(uint8_t)std::round(Clamp01(c.g) * 255.f),
			(uint8_t)std::round(Clamp01(c.b) * 255.f),
			(uint8_t)std::round(Clamp01(c.a) * 255.f)
		);
	}

	inline constexpr Vector2 ToVector2(const Vector2Int& v)
	{
		return Vector2((float)v.x, (float)v.y);
	}

	inline constexpr Vector2 ToVector2(const Vector3& v)
	{
		return Vector2(v.x, v.y);
	}

	inline constexpr Vector2 ToVector2(const Vector4& v)
	{
		return Vector2(v.x, v.y);
	}

	inline constexpr Vector2Int ToVector2Int(const Vector3Int& v)
	{
		return Vector2Int(v.x, v.y);
	}

	inline constexpr Vector3 ToVector3(const Vector3Int& v)
	{
		return Vector3((float)v.x, (float)v.y, (float)v.z);
	}

	inline constexpr Vector3 ToVector3(const Vector2& v)
	{
		return Vector3(v.x, v.y, 0.f);
	}

	inline constexpr Vector3 ToVector3(const Vector4& v)
	{
		return Vector3(v.x, v.y, v.z);
	}

	inline constexpr Vector3Int ToVector3Int(const Vector2Int& v)
	{
		return Vector3Int(v.x, v.y, 0);
	}

	inline constexpr Vector4 ToVector4(const Color& c)
	{
		return Vector4(c.r, c.g, c.b, c.a);
	}

	inline constexpr Vector4 ToVector4(const Vector2& v)
	{
		return Vector4(v.x, v.y, 0.f, 0.f);
	}

	inline constexpr Vector4 ToVector4(const Vector3& v)
	{
		return Vector4(v.x, v.y, v.z, 0.f);
	}
}

