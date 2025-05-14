#pragma once

#include <cmath>
#include <cstdint>

#include "utilities.h"
#include "vector2.h"
#include "vector3.h"
#include "vector4.h"

namespace agm
{
	inline constexpr Vector2 ToVector2(const Vector3& v)
	{
		return Vector2(v.x, v.y);
	}

	inline constexpr Vector2 ToVector2(const Vector4& v)
	{
		return Vector2(v.x, v.y);
	}

	inline constexpr Vector3 ToVector3(const Vector2& v)
	{
		return Vector3(v.x, v.y, 0.f);
	}

	inline constexpr Vector3 ToVector3(const Vector4& v)
	{
		return Vector3(v.x, v.y, v.z);
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

