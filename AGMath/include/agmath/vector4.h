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
			return Vector4(v.x * scalar, v.y * scalar, v.z * scalar, v.w * scalar);
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

		void Normalize(float tolerance = EPSILON)
		{
			*this = GetNormalized(tolerance);
		}

		Vector4 GetNormalized(float tolerance = EPSILON) const
		{
			float lengthSq = LengthSquared();
			if (lengthSq > tolerance)
			{
				return *this * agm::InvSqrt(lengthSq);
			}
			else
			{
				return Vector4::ZERO;
			}
		}

		constexpr bool IsNormalized(float threshold = agm::VECTOR_NORMALIZED_THRESHOLD) const
		{
			return Abs(1.f - LengthSquared()) < threshold;
		}

		constexpr Vector4 GetAbs() const
		{
			return Vector4(agm::Abs(x), agm::Abs(y), agm::Abs(z), agm::Abs(w));
		}

		constexpr float GetMax() const
		{
			return agm::Max(agm::Max(x, y), agm::Max(z, w));
		}

		constexpr float GetAbsMax() const
		{
			return agm::Max(agm::Max(agm::Abs(x), agm::Abs(y)), agm::Max(agm::Abs(z), agm::Abs(w)));
		}

		constexpr float GetMin() const
		{
			return agm::Min(agm::Min(x, y), agm::Min(z, w));
		}

		constexpr float GetAbsMin() const
		{
			return agm::Min(agm::Min(agm::Abs(x), agm::Abs(y)), agm::Min(agm::Abs(z), agm::Abs(w)));
		}

		constexpr bool IsNearlyZero(float tolerance = EPSILON) const
		{
			return
				agm::IsNearlyZero(x, tolerance) &&
				agm::IsNearlyZero(y, tolerance) &&
				agm::IsNearlyZero(z, tolerance) &&
				agm::IsNearlyZero(w, tolerance);
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

		constexpr bool Equals(const Vector4& other, float tolerance = EPSILON) const
		{
			return
				agm::IsNearlyEqual(x, other.x, tolerance) &&
				agm::IsNearlyEqual(y, other.y, tolerance) &&
				agm::IsNearlyEqual(z, other.z, tolerance) &&
				agm::IsNearlyEqual(w, other.w, tolerance);
		}

		std::string ToString() const
		{
			return std::format("({:.2f}, {:.2f}, {:.2f}, {:.2f})", x, y, z, w);
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
			float clampedT = agm::Clamp01(t);
			return Vector4(
				a.x + (b.x - a.x) * clampedT,
				a.y + (b.y - a.y) * clampedT,
				a.z + (b.z - a.z) * clampedT,
				a.w + (b.w - a.w) * clampedT
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

		static Vector4 MoveTowards(const Vector4& current, const Vector4& target, float maxDistanceDelta)
		{
			float effMaxDistanceDelta = maxDistanceDelta < 0.f ? 0.f : maxDistanceDelta;
			Vector4 delta = target - current;
			float distSq = delta.LengthSquared();
			if (agm::IsNearlyZero(distSq) || distSq <= effMaxDistanceDelta * effMaxDistanceDelta)
			{
				return target;
			}

			return current + delta * (effMaxDistanceDelta / agm::Sqrt(distSq));
		}

		static constexpr Vector4 Project(const Vector4& v, const Vector4& onNormal)
		{
			float normalLengthSq = Dot(onNormal, onNormal);
			if (agm::IsNearlyZero(normalLengthSq))
			{
				return Vector4::ZERO;
			}

			return onNormal * (Dot(v, onNormal) / normalLengthSq);
		}
	};

	inline const Vector4 Vector4::ZERO = Vector4(0.f, 0.f, 0.f, 0.f);
	inline const Vector4 Vector4::ONE = Vector4(1.f, 1.f, 1.f, 1.f);
}

