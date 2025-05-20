#pragma once

#include <cmath>
#include <cstdint>
#include <format>
#include <stdexcept>
#include <string>

#include "utilities.h"

namespace agm
{
	struct Vector2
	{
	public:

		float x;
		float y;

	public:

		static const Vector2 ZERO;
		static const Vector2 ONE;
		static const Vector2 UP;
		static const Vector2 DOWN;
		static const Vector2 LEFT;
		static const Vector2 RIGHT;

	public:

		constexpr Vector2()
			: x(0.f)
			, y(0.f)
		{
		}

		constexpr Vector2(float x, float y)
			: x(x)
			, y(y)
		{
		}

	public:

		float operator[](int32_t index) const
		{
			switch (index)
			{
			case 0: return x;
			case 1: return y;
			default: throw std::out_of_range("Invalid Vector2 index!");
			}
		}

		float& operator[](int32_t index)
		{
			switch (index)
			{
			case 0: return x;
			case 1: return y;
			default: throw std::out_of_range("Invalid Vector2 index!");
			}
		}

		constexpr Vector2 operator-() const
		{
			return Vector2(-x, -y);
		}

		constexpr Vector2 operator*(float scalar) const
		{
			return Vector2(x * scalar, y * scalar);
		}

		constexpr Vector2 operator/(float scalar) const
		{
			return Vector2(x / scalar, y / scalar);
		}

		constexpr Vector2 operator+(const Vector2& other) const
		{
			return Vector2(x + other.x, y + other.y);
		}

		constexpr Vector2 operator-(const Vector2& other) const
		{
			return Vector2(x - other.x, y - other.y);
		}

		constexpr Vector2 operator*(const Vector2& other) const
		{
			return Vector2(x * other.x, y * other.y);
		}

		constexpr Vector2 operator/(const Vector2& other) const
		{
			return Vector2(x / other.x, y / other.y);
		}

		constexpr Vector2& operator*=(float scalar)
		{
			x *= scalar;
			y *= scalar;
			return *this;
		}

		constexpr Vector2& operator/=(float scalar)
		{
			x /= scalar;
			y /= scalar;
			return *this;
		}

		constexpr Vector2& operator+=(const Vector2& other)
		{
			x += other.x;
			y += other.y;
			return *this;
		}

		constexpr Vector2& operator-=(const Vector2& other)
		{
			x -= other.x;
			y -= other.y;
			return *this;
		}

		constexpr Vector2& operator*=(const Vector2& other)
		{
			x *= other.x;
			y *= other.y;
			return *this;
		}

		constexpr Vector2& operator/=(const Vector2& other)
		{
			x /= other.x;
			y /= other.y;
			return *this;
		}

		constexpr bool operator==(const Vector2& other) const
		{
			return x == other.x && y == other.y;
		}

		constexpr bool operator!=(const Vector2& other) const
		{
			return !(*this == other);
		}

		friend constexpr Vector2 operator*(float scalar, const Vector2& v)
		{
			return Vector2(v.x * scalar, v.y * scalar);
		}

	public:

		float Length() const
		{
			return std::sqrt(x * x + y * y);
		}

		constexpr float LengthSquared() const
		{
			return x * x + y * y;
		}

		void Normalize(float tolerance = EPSILON)
		{
			*this = GetNormalized(tolerance);
		}

		Vector2 GetNormalized(float tolerance = EPSILON) const
		{
			float lengthSq = LengthSquared();
			if (lengthSq > tolerance)
			{
				float invLength = 1.f / std::sqrt(lengthSq);
				return *this * invLength;
			}
			else
			{
				return Vector2::ZERO;
			}
		}

		constexpr bool IsNormalized() const
		{
			return Abs(1.f - LengthSquared()) < VECTOR_NORMALIZED_THRESHOLD;
		}

		constexpr Vector2 GetAbs() const
		{
			return Vector2(Abs(x), Abs(y));
		}

		constexpr float GetMax() const
		{
			return agm::Max(x, y);
		}

		constexpr float GetAbsMax() const
		{
			return agm::Max(Abs(x), Abs(y));
		}

		constexpr float GetMin() const
		{
			return agm::Min(x, y);
		}

		constexpr float GetAbsMin() const
		{
			return agm::Min(Abs(x), Abs(y));
		}

		constexpr bool IsNearlyZero(float tolerance = EPSILON) const
		{
			return Abs(x) <= tolerance && Abs(y) <= tolerance;
		}

		constexpr bool IsZero() const
		{
			return x == 0.f && y == 0.f;
		}

		constexpr void Set(float x, float y)
		{
			this->x = x;
			this->y = y;
		}

		constexpr bool Equals(const Vector2& other, float tolerance = EPSILON) const
		{
			return Abs(x - other.x) <= tolerance && Abs(y - other.y) <= tolerance;
		}

		std::string ToString() const
		{
			return std::format("({:.2f}, {:.2f})", x, y);
		}

	public:

		static float Angle(const Vector2& from, const Vector2& to)
		{
			float denominator = std::sqrt(from.LengthSquared() * to.LengthSquared());
			if (agm::IsNearlyZero(denominator))
			{
				return 0.f;
			}

			float cosTheta = Dot(from, to) / denominator;
			return std::acos(agm::Clamp(cosTheta, -1.f, 1.f)) * agm::RAD2DEG;
		}

		static Vector2 Clamp(const Vector2& v, const Vector2& min, const Vector2& max)
		{
			return Vector2(agm::Clamp(v.x, min.x, max.x), agm::Clamp(v.y, min.y, max.y));
		}

		static Vector2 ClampLength(const Vector2& v, float maxLength)
		{
			float lengthSq = v.LengthSquared();
			if (lengthSq > maxLength * maxLength)
			{
				float length = std::sqrt(lengthSq);
				return v * (maxLength / length);
			}

			return v;
		}

		static constexpr float Cross(const Vector2& a, const Vector2& b)
		{
			return a.x * b.y - a.y * b.x;
		}

		static float Distance(const Vector2& a, const Vector2& b)
		{
			return (a - b).Length();
		}

		static constexpr float DistanceSquared(const Vector2& a, const Vector2& b)
		{
			return (a - b).LengthSquared();
		}

		static constexpr float Dot(const Vector2& a, const Vector2& b)
		{
			return a.x * b.x + a.y * b.y;
		}

		static constexpr Vector2 Lerp(const Vector2& a, const Vector2& b, float t)
		{
			t = Clamp01(t);
			return Vector2(a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t);
		}

		static constexpr Vector2 Max(const Vector2& a, const Vector2& b)
		{
			return Vector2(agm::Max(a.x, b.x), agm::Max(a.y, b.y));
		}

		static constexpr Vector2 Min(const Vector2& a, const Vector2& b)
		{
			return Vector2(agm::Min(a.x, b.x), agm::Min(a.y, b.y));
		}

		static Vector2 MoveTowards(const Vector2& current, const Vector2& target, float maxDistanceDelta)
		{
			Vector2 delta = target - current;
			float lengthSq = delta.LengthSquared();
			if (agm::IsNearlyZero(lengthSq) || (maxDistanceDelta >= 0.f && lengthSq <= maxDistanceDelta * maxDistanceDelta))
			{
				return target;
			}

			float length = std::sqrt(lengthSq);
			return current + delta / length * maxDistanceDelta;
		}

		static constexpr Vector2 Perpendicular(const Vector2& v, bool clockwise = false)
		{
			return clockwise ? Vector2(v.y, -v.x) : Vector2(-v.y, v.x);
		}

		static constexpr Vector2 Project(const Vector2& v, const Vector2& onNormal)
		{
			float normalLengthSq = onNormal.LengthSquared();
			if (agm::IsNearlyZero(normalLengthSq))
			{
				return Vector2::ZERO;
			}

			return (Dot(v, onNormal) / normalLengthSq) * onNormal;
		}

		static constexpr Vector2 Reflect(const Vector2& inDirection, const Vector2& inNormal)
		{
			return inDirection - 2.f * Dot(inDirection, inNormal) * inNormal;
		}

		static float SignedAngle(const Vector2& from, const Vector2& to)
		{
			float angle = Angle(from, to);
			float sign = Sign(from.x * to.y - from.y * to.x);
			return angle * sign;
		}
	};

	inline const Vector2 Vector2::ZERO = Vector2(0.f, 0.f);
	inline const Vector2 Vector2::ONE = Vector2(1.f, 1.f);
	inline const Vector2 Vector2::UP = Vector2(0.f, 1.f);
	inline const Vector2 Vector2::DOWN = Vector2(0.f, -1.f);
	inline const Vector2 Vector2::LEFT = Vector2(-1.f, 0.f);
	inline const Vector2 Vector2::RIGHT = Vector2(1.f, 0.f);
}

