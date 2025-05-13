#pragma once

#include <cmath>
#include <cstdint>
#include <format>
#include <stdexcept>
#include <string>

#include "utilities.h"

namespace agm
{
	struct Vector4
	{
	public:

		float x;
		float y;
		float z;
		float w;

	public:

		static const Vector4 ZERO;
		static const Vector4 ONE;

	public:

		constexpr Vector4()
			: x(0.f)
			, y(0.f)
			, z(0.f)
			, w(0.f)
		{
		}

		constexpr Vector4(float x, float y, float z, float w)
			: x(x)
			, y(y)
			, z(z)
			, w(w)
		{
		}

	public:

		constexpr float operator[](int32_t index) const
		{
			switch (index)
			{
			case 0: return x;
			case 1: return y;
			case 2: return z;
			case 3: return w;
			default: throw std::out_of_range("Invalid Vector4 index!");
			}
		}

		constexpr float& operator[](int32_t index)
		{
			switch (index)
			{
			case 0: return x;
			case 1: return y;
			case 2: return z;
			case 3: return w;
			default: throw std::out_of_range("Invalid Vector4 index!");
			}
		}

		constexpr Vector4 operator-() const
		{
			return Vector4(-x, -y, -z, -w);
		}

		constexpr Vector4 operator*(float scalar) const
		{
			return Vector4(x * scalar, y * scalar, z * scalar, w * scalar);
		}

		constexpr Vector4 operator/(float scalar) const
		{
			return Vector4(x / scalar, y / scalar, z / scalar, w / scalar);
		}

		constexpr Vector4 operator+(const Vector4& other) const
		{
			return Vector4(x + other.x, y + other.y, z + other.z, w + other.w);
		}

		constexpr Vector4 operator-(const Vector4& other) const
		{
			return Vector4(x - other.x, y - other.y, z - other.z, w - other.w);
		}

		constexpr Vector4 operator*(const Vector4& other) const
		{
			return Vector4(x * other.x, y * other.y, z * other.z, w * other.w);
		}

		constexpr Vector4 operator/(const Vector4& other) const
		{
			return Vector4(x / other.x, y / other.y, z / other.z, w / other.w);
		}

		constexpr Vector4& operator*=(float scalar)
		{
			x *= scalar;
			y *= scalar;
			z *= scalar;
			w *= scalar;
			return *this;
		}

		constexpr Vector4& operator/=(float scalar)
		{
			x /= scalar;
			y /= scalar;
			z /= scalar;
			w /= scalar;
			return *this;
		}

		constexpr Vector4& operator+=(const Vector4& other)
		{
			x += other.x;
			y += other.y;
			z += other.z;
			w += other.w;
			return *this;
		}

		constexpr Vector4& operator-=(const Vector4& other)
		{
			x -= other.x;
			y -= other.y;
			z -= other.z;
			w -= other.w;
			return *this;
		}

		constexpr Vector4& operator*=(const Vector4& other)
		{
			x *= other.x;
			y *= other.y;
			z *= other.z;
			w *= other.w;
			return *this;
		}

		constexpr Vector4& operator/=(const Vector4& other)
		{
			x /= other.x;
			y /= other.y;
			z /= other.z;
			w /= other.w;
			return *this;
		}

		constexpr bool operator==(const Vector4& other) const
		{
			return x == other.x && y == other.y && z == other.z && w == other.w;
		}

		constexpr bool operator!=(const Vector4& other) const
		{
			return !(*this == other);
		}

		friend constexpr Vector4 operator*(float scalar, const Vector4& v)
		{
			return Vector4(v.x * scalar, v.y * scalar, v.z * scalar, v.w * scalar);
		}

	public:

		float Length() const
		{
			return std::sqrt(x * x + y * y + z * z + w * w);
		}

		constexpr float LengthSquared() const
		{
			return x * x + y * y + z * z + w * w;
		}

		void Normalize(float tolerance = EPSILON)
		{
			*this = GetNormalized(tolerance);
		}

		Vector4 GetNormalized(float tolerance = EPSILON) const
		{
			float lengthSq = LengthSquared();
			if (lengthSq > tolerance)
			{
				float invLength = 1.f / std::sqrt(lengthSq);
				return *this * invLength;
			}
			else
			{
				return Vector4::ZERO;
			}
		}

		constexpr bool IsNormalized() const
		{
			return Abs(1.f - LengthSquared()) < THRESH_VECTOR_NORMALIZED;
		}

		constexpr Vector4 GetAbs() const
		{
			return Vector4(Abs(x), Abs(y), Abs(z), Abs(w));
		}

		constexpr float GetMax() const
		{
			return agm::Max(agm::Max(x, y), agm::Max(z, w));
		}

		constexpr float GetAbsMax() const
		{
			return agm::Max(agm::Max(Abs(x), Abs(y)), agm::Max(Abs(z), Abs(w)));
		}

		constexpr float GetMin() const
		{
			return agm::Min(agm::Min(x, y), agm::Min(z, w));
		}

		constexpr float GetAbsMin() const
		{
			return agm::Min(agm::Min(Abs(x), Abs(y)), agm::Min(Abs(z), Abs(w)));
		}

		constexpr bool IsNearlyZero(float tolerance = LOOSE_EPSILON) const
		{
			return Abs(x) <= tolerance && Abs(y) <= tolerance && Abs(z) <= tolerance && Abs(w) <= tolerance;
		}

		constexpr bool IsZero() const
		{
			return x == 0.f && y == 0.f && z == 0.f && w == 0.f;
		}

		constexpr void Set(float x, float y, float z, float w)
		{
			this->x = x;
			this->y = y;
			this->z = z;
			this->w = w;
		}

		constexpr bool Equals(const Vector4& other, float tolerance = LOOSE_EPSILON) const
		{
			return Abs(x - other.x) <= tolerance && Abs(y - other.y) <= tolerance && Abs(z - other.z) <= tolerance && Abs(w - other.w) <= tolerance;
		}

		std::string ToString() const
		{
			return std::format("({:.2f}, {:.2f}, {:.2f}, {:.2f})", x, y, z, w);
		}

	public:

		static float Distance(const Vector4& a, const Vector4& b)
		{
			return (a - b).Length();
		}

		static constexpr float Dot(const Vector4& a, const Vector4& b)
		{
			return a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
		}

		static constexpr Vector4 Lerp(const Vector4& a, const Vector4& b, float t)
		{
			t = Clamp01(t);
			return Vector4(a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t, a.z + (b.z - a.z) * t, a.w + (b.w - a.w) * t);
		}

		static constexpr Vector4 LerpUnclamped(const Vector4& a, const Vector4& b, float t)
		{
			return Vector4(a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t, a.z + (b.z - a.z) * t, a.w + (b.w - a.w) * t);
		}

		static constexpr Vector4 Max(const Vector4& a, const Vector4& b)
		{
			return Vector4(agm::Max(a.x, b.x), agm::Max(a.y, b.y), agm::Max(a.z, b.z), agm::Max(a.w, b.w));
		}

		static constexpr Vector4 Min(const Vector4& a, const Vector4& b)
		{
			return Vector4(agm::Min(a.x, b.x), agm::Min(a.y, b.y), agm::Min(a.z, b.z), agm::Min(a.w, b.w));
		}

		static Vector4 MoveTowards(const Vector4& current, const Vector4& target, float maxDistanceDelta)
		{
			Vector4 direction = target - current;
			float lengthSq = direction.LengthSquared();
			if (agm::IsNearlyZero(lengthSq) || (maxDistanceDelta >= 0.f && lengthSq <= maxDistanceDelta * maxDistanceDelta))
			{
				return target;
			}

			float length = std::sqrt(lengthSq);
			return current + direction / length * maxDistanceDelta;
		}

		static constexpr Vector4 Project(const Vector4& a, const Vector4& b)
		{
			return b * (Dot(a, b) / Dot(b, b));
		}
	};

	inline const Vector4 Vector4::ZERO = Vector4(0.f, 0.f, 0.f, 0.f);
	inline const Vector4 Vector4::ONE = Vector4(1.f, 1.f, 1.f, 1.f);
}

