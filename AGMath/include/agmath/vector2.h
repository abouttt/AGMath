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

		constexpr Vector2(float fill)
			: x(fill)
			, y(fill)
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
			return v * scalar;
		}

	public:

		float Length() const
		{
			return agm::Sqrt(x * x + y * y);
		}

		constexpr float LengthSquared() const
		{
			return x * x + y * y;
		}

		void Normalize(float tolerance = agm::EPSILON_SQ_LENGTH)
		{
			*this = GetNormalized(tolerance);
		}

		Vector2 GetNormalized(float tolerance = agm::EPSILON_SQ_LENGTH) const
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
			return agm::Abs(x) <= tolerance && agm::Abs(y) <= tolerance;
		}

		constexpr bool IsZero() const
		{
			return x == 0.f && y == 0.f;
		}

		constexpr Vector2 GetAbs() const
		{
			return Vector2(agm::Abs(x), agm::Abs(y));
		}

		constexpr float GetMax() const
		{
			return agm::Max(x, y);
		}

		constexpr float GetMin() const
		{
			return agm::Min(x, y);
		}

		constexpr void Set(float x, float y)
		{
			this->x = x;
			this->y = y;
		}

		constexpr bool Equals(const Vector2& other, float tolerance = agm::EPSILON) const
		{
			return agm::Abs(x - other.x) <= tolerance && agm::Abs(y - other.y) <= tolerance;
		}

		std::string ToString() const
		{
			return std::format("({:.3f}, {:.3f})", x, y);
		}

	public:

		static float Angle(const Vector2& a, const Vector2& b)
		{
			float denominatorSq = a.LengthSquared() * b.LengthSquared();
			if (agm::IsNearlyZero(denominatorSq, agm::EPSILON_SQ_LENGTH))
			{
				return 0.f;
			}

			float cosTheta = Dot(a, b) / agm::Sqrt(denominatorSq);
			return agm::Acos(agm::Clamp(cosTheta, -1.f, 1.f)) * agm::RAD2DEG;
		}

		static constexpr Vector2 Clamp(const Vector2& v, const Vector2& min, const Vector2& max)
		{
			return Vector2(agm::Clamp(v.x, min.x, max.x), agm::Clamp(v.y, min.y, max.y));
		}

		static Vector2 ClampLength(const Vector2& v, float maxLength)
		{
			if (maxLength < 0.f)
			{
				maxLength = 0.f;
			}

			float lenSq = v.LengthSquared();
			if (lenSq > maxLength * maxLength && !agm::IsNearlyZero(lenSq, agm::EPSILON_SQ_LENGTH))
			{
				return v * (maxLength / agm::Sqrt(lenSq));
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
			t = agm::Clamp01(t);
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

		static Vector2 MoveTowards(const Vector2& current, const Vector2& target, float maxDelta)
		{
			if (maxDelta < 0.f)
			{
				maxDelta = 0.f;
			}

			Vector2 delta = target - current;
			float distSq = delta.LengthSquared();
			if (distSq <= maxDelta * maxDelta || agm::IsNearlyZero(distSq, agm::EPSILON_SQ_LENGTH))
			{
				return target;
			}

			return current + delta * (maxDelta / agm::Sqrt(distSq));
		}

		static constexpr Vector2 Perpendicular(const Vector2& v, bool clockwise = false)
		{
			return clockwise ? Vector2(v.y, -v.x) : Vector2(-v.y, v.x);
		}

		static constexpr Vector2 Project(const Vector2& v, const Vector2& onNormal)
		{
			float normalLenSq = Dot(onNormal, onNormal);
			if (agm::IsNearlyZero(normalLenSq, agm::EPSILON_SQ_LENGTH))
			{
				return ZERO;
			}

			return onNormal * (Dot(v, onNormal) / normalLenSq);
		}

		static constexpr Vector2 Reflect(const Vector2& inDirection, const Vector2& inNormal)
		{
			float normalLenSq = Dot(inNormal, inNormal);
			if (agm::IsNearlyZero(normalLenSq, agm::EPSILON_SQ_LENGTH))
			{
				return inDirection;
			}

			return inDirection - (2.f * Dot(inDirection, inNormal) / normalLenSq) * inNormal;
		}

		static float SignedAngle(const Vector2& a, const Vector2& b)
		{
			float angle = Angle(a, b);
			float sign = Cross(a, b);
			return angle * agm::Sign(sign);
		}
	};

	inline const Vector2 Vector2::ZERO = Vector2(0.f, 0.f);
	inline const Vector2 Vector2::ONE = Vector2(1.f, 1.f);
	inline const Vector2 Vector2::UP = Vector2(0.f, 1.f);
	inline const Vector2 Vector2::DOWN = Vector2(0.f, -1.f);
	inline const Vector2 Vector2::LEFT = Vector2(-1.f, 0.f);
	inline const Vector2 Vector2::RIGHT = Vector2(1.f, 0.f);
}

