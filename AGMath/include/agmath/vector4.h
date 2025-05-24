#pragma once

#include <cmath>
#include <cstdint>
#include <format>
#include <stdexcept>
#include <string>

#include "utilities.h"
#include "vector3.h"

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

		constexpr Vector4(float fill)
			: x(fill)
			, y(fill)
			, z(fill)
			, w(fill)
		{
		}

		constexpr Vector4(float x, float y, float z, float w)
			: x(x)
			, y(y)
			, z(z)
			, w(w)
		{
		}

		constexpr explicit Vector4(const Vector3& v3, float w = 0.f)
			: x(v3.x)
			, y(v3.y)
			, z(v3.z)
			, w(0.f)
		{
		}

	public:

		float operator[](int32_t index) const
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

		float& operator[](int32_t index)
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
			return v * scalar;
		}

	public:

		float Length() const
		{
			return agm::Sqrt(x * x + y * y + z * z + w * w);
		}

		constexpr float LengthSquared() const
		{
			return x * x + y * y + z * z + w * w;
		}

		void Normalize(float tolerance = agm::EPSILON_SQ_LENGTH)
		{
			*this = GetNormalized(tolerance);
		}

		Vector4 GetNormalized(float tolerance = agm::EPSILON_SQ_LENGTH) const
		{
			float lenSq = LengthSquared();
			if (lenSq > tolerance)
			{
				return *this * agm::InvSqrt(lenSq);
			}
			else
			{
				return ZERO;
			}
		}

		bool IsNormalized(float threshold = agm::VECTOR_NORMALIZED_THRESHOLD) const
		{
			return agm::Abs(1.f - LengthSquared()) < threshold;
		}

		constexpr bool IsNearlyZero(float tolerance = agm::EPSILON) const
		{
			return agm::Abs(x) < tolerance &&
				   agm::Abs(y) < tolerance && 
				   agm::Abs(z) < tolerance && 
				   agm::Abs(w) < tolerance;
		}

		constexpr bool IsZero() const
		{
			return x == 0.f && y == 0.f && z == 0.f && w == 0.f;
		}

		constexpr Vector4 GetAbs() const
		{
			return Vector4(agm::Abs(x), agm::Abs(y), agm::Abs(z), agm::Abs(w));
		}

		constexpr float GetMax() const
		{
			return agm::Max(agm::Max(x, y), agm::Max(z, w));
		}

		constexpr float GetMin() const
		{
			return agm::Min(agm::Min(x, y), agm::Min(z, w));
		}

		constexpr void Set(float x, float y, float z, float w)
		{
			this->x = x;
			this->y = y;
			this->z = z;
			this->w = w;
		}

		constexpr bool Equals(const Vector4& other, float tolerance = agm::EPSILON) const
		{
			return agm::Abs(x - other.x) < tolerance &&
				   agm::Abs(y - other.y) < tolerance &&
				   agm::Abs(z - other.z) < tolerance &&
				   agm::Abs(w - other.w) < tolerance;
		}

		std::string ToString() const
		{
			return std::format("({:.3f}, {:.3f}, {:.3f}, {:.3f})", x, y, z, w);
		}

		constexpr Vector3 ToVector3() const
		{
			return Vector3(x, y, z);
		}

	public:

		static constexpr Vector4 Clamp(const Vector4& v, const Vector4& min, const Vector4& max)
		{
			return Vector4(
				agm::Clamp(v.x, min.x, max.x),
				agm::Clamp(v.y, min.y, max.y),
				agm::Clamp(v.z, min.z, max.z),
				agm::Clamp(v.w, min.w, max.w)
			);
		}

		static float Distance(const Vector4& a, const Vector4& b)
		{
			return (a - b).Length();
		}

		static constexpr float DistanceSquared(const Vector4& a, const Vector4& b)
		{
			return (a - b).LengthSquared();
		}

		static constexpr float Dot(const Vector4& a, const Vector4& b)
		{
			return a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
		}

		static constexpr Vector4 Lerp(const Vector4& a, const Vector4& b, float t)
		{
			t = agm::Clamp01(t);
			return Vector4(
				a.x + (b.x - a.x) * t,
				a.y + (b.y - a.y) * t,
				a.z + (b.z - a.z) * t,
				a.w + (b.w - a.w) * t
			);
		}

		static constexpr Vector4 Max(const Vector4& a, const Vector4& b)
		{
			return Vector4(agm::Max(a.x, b.x), agm::Max(a.y, b.y), agm::Max(a.z, b.z), agm::Max(a.w, b.w));
		}

		static constexpr Vector4 Min(const Vector4& a, const Vector4& b)
		{
			return Vector4(agm::Min(a.x, b.x), agm::Min(a.y, b.y), agm::Min(a.z, b.z), agm::Min(a.w, b.w));
		}

		static Vector4 MoveTowards(const Vector4& current, const Vector4& target, float maxDelta)
		{
			if (maxDelta < 0.f)
			{
				maxDelta = 0.f;
			}

			Vector4 delta = target - current;
			float distSq = delta.LengthSquared();
			if (distSq <= maxDelta * maxDelta || agm::IsNearlyZero(distSq, agm::EPSILON_SQ_LENGTH))
			{
				return target;
			}

			return current + delta * (maxDelta / agm::Sqrt(distSq));
		}

		static constexpr Vector4 Project(const Vector4& v, const Vector4& onNormal)
		{
			float normalLenSq = Dot(onNormal, onNormal);
			if (agm::IsNearlyZero(normalLenSq, agm::EPSILON_SQ_LENGTH))
			{
				return ZERO;
			}

			return onNormal * (Dot(v, onNormal) / normalLenSq);
		}
	};

	inline const Vector4 Vector4::ZERO = Vector4(0.f, 0.f, 0.f, 0.f);
	inline const Vector4 Vector4::ONE = Vector4(1.f, 1.f, 1.f, 1.f);
}

