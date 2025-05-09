#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <format>
#include <string>

#include "utilities.h"

namespace agm
{
	struct Vector2
	{
		static const Vector2 ZERO;
		static const Vector2 ONE;
		static const Vector2 UP;
		static const Vector2 DOWN;
		static const Vector2 LEFT;
		static const Vector2 RIGHT;

		union
		{
			struct
			{
				float x;
				float y;
			};

			std::array<float, 2> data;
		};

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

		inline constexpr float operator[](size_t index) const
		{
			return data[index];
		}

		inline constexpr float& operator[](size_t index)
		{
			return data[index];
		}

		inline constexpr Vector2 operator-() const
		{
			return Vector2(-x, -y);
		}

		inline constexpr Vector2 operator*(float scalar) const
		{
			return Vector2(x * scalar, y * scalar);
		}

		inline constexpr Vector2 operator/(float scalar) const
		{
			return Vector2(x / scalar, y / scalar);
		}

		inline constexpr Vector2 operator+(const Vector2& other) const
		{
			return Vector2(x + other.x, y + other.y);
		}

		inline constexpr Vector2 operator-(const Vector2& other) const
		{
			return Vector2(x - other.x, y - other.y);
		}

		inline constexpr Vector2 operator*(const Vector2& other) const
		{
			return Vector2(x * other.x, y * other.y);
		}

		inline constexpr Vector2 operator/(const Vector2& other) const
		{
			return Vector2(x / other.x, y / other.y);
		}

		inline constexpr Vector2& operator*=(float scalar)
		{
			x *= scalar;
			y *= scalar;
			return *this;
		}

		inline constexpr Vector2& operator/=(float scalar)
		{
			x /= scalar;
			y /= scalar;
			return *this;
		}

		inline constexpr Vector2& operator+=(const Vector2& other)
		{
			x += other.x;
			y += other.y;
			return *this;
		}

		inline constexpr Vector2& operator-=(const Vector2& other)
		{
			x -= other.x;
			y -= other.y;
			return *this;
		}

		inline constexpr Vector2& operator*=(const Vector2& other)
		{
			x *= other.x;
			y *= other.y;
			return *this;
		}

		inline constexpr Vector2& operator/=(const Vector2& other)
		{
			x /= other.x;
			y /= other.y;
			return *this;
		}

		inline constexpr bool operator==(const Vector2& other) const
		{
			return x == other.x && y == other.y;
		}

		inline constexpr bool operator!=(const Vector2& other) const
		{
			return !(*this == other);
		}

		friend inline constexpr Vector2 operator*(float scalar, const Vector2& v)
		{
			return Vector2(v.x * scalar, v.y * scalar);
		}

		static inline float Angle(const Vector2& from, const Vector2& to)
		{
			float length = std::sqrt(from.LengthSquared() * to.LengthSquared());
			if (agm::IsNearlyZero(length))
			{
				return 0.f;
			}

			float cosTheta = Clamp(Dot(from, to) / length, -1.f, 1.f);
			return std::acos(cosTheta) * RAD2DEG;
		}

		static inline Vector2 ClampLength(const Vector2& v, float maxLength)
		{
			float lengthSq = v.LengthSquared();
			if (lengthSq > maxLength * maxLength)
			{
				float length = std::sqrt(lengthSq);
				Vector2 norm = v / length;
				return norm * maxLength;
			}

			return v;
		}

		static inline float Distance(const Vector2& a, const Vector2& b)
		{
			return (a - b).Length();
		}

		static inline constexpr float Dot(const Vector2& a, const Vector2& b)
		{
			return a.x * b.x + a.y * b.y;
		}

		static inline constexpr Vector2 Lerp(const Vector2& a, const Vector2& b, float t)
		{
			t = Clamp01(t);
			return Vector2(a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t);
		}

		static inline constexpr Vector2 LerpUnclamped(const Vector2& a, const Vector2& b, float t)
		{
			return Vector2(a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t);
		}

		static inline constexpr Vector2 Max(const Vector2& a, const Vector2& b)
		{
			return Vector2(agm::Max(a.x, b.x), agm::Max(a.y, b.y));
		}

		static inline constexpr Vector2 Min(const Vector2& a, const Vector2& b)
		{
			return Vector2(agm::Min(a.x, b.x), agm::Min(a.y, b.y));
		}

		static inline Vector2 MoveTowards(const Vector2& current, const Vector2& target, float maxDistanceDelta)
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

		static inline constexpr Vector2 Perpendicular(const Vector2& v)
		{
			return Vector2(-v.y, v.x);
		}

		static inline constexpr Vector2 Project(const Vector2& a, const Vector2& b)
		{
			float denom = Dot(b, b);
			if (agm::IsNearlyZero(denom))
			{
				return Vector2::ZERO;
			}

			return b * (Dot(a, b) / denom);
		}

		static inline constexpr Vector2 Reflect(const Vector2& inDirection, const Vector2& inNormal)
		{
			return (-2.f * Dot(inDirection, inNormal)) * inNormal + inDirection;
		}

		static inline Vector2 Rotate(const Vector2& v, float angle)
		{
			float rad = angle * DEG2RAD;
			float cosA = std::cos(rad);
			float sinA = std::sin(rad);
			return Vector2(v.x * cosA - v.y * sinA, v.x * sinA + v.y * cosA);
		}

		static inline float SignedAngle(const Vector2& from, const Vector2& to)
		{
			return Angle(from, to) * Sign(from.x * to.y - from.y * to.x);
		}

		inline constexpr bool Equals(const Vector2& other, float tolerance = LOOSE_EPSILON) const
		{
			return Abs(x - other.x) <= tolerance && Abs(y - other.y) <= tolerance;
		}

		inline constexpr bool IsNearlyZero(float tolerance = LOOSE_EPSILON) const
		{
			return Abs(x) <= tolerance && Abs(y) <= tolerance;
		}

		inline constexpr bool IsZero() const
		{
			return x == 0.f && y == 0.f;
		}

		inline constexpr bool IsNormalized() const
		{
			return Abs(1.f - LengthSquared()) < 0.01f;
		}

		inline float Length() const
		{
			return std::sqrt(x * x + y * y);
		}

		inline constexpr float LengthSquared() const
		{
			return x * x + y * y;
		}

		inline void Normalize(float tolerance = EPSILON)
		{
			*this = Normalized(tolerance);
		}

		inline Vector2 Normalized(float tolerance = EPSILON) const
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

		inline constexpr void Set(float x, float y)
		{
			this->x = x;
			this->y = y;
		}

		std::string ToString() const
		{
			return std::format("({:.2f}, {:.2f})", x, y);
		}
	};

	inline const Vector2 Vector2::ZERO = Vector2(0.f, 0.f);
	inline const Vector2 Vector2::ONE = Vector2(1.f, 1.f);
	inline const Vector2 Vector2::UP = Vector2(0.f, 1.f);
	inline const Vector2 Vector2::DOWN = Vector2(0.f, -1.f);
	inline const Vector2 Vector2::LEFT = Vector2(-1.f, 0.f);
	inline const Vector2 Vector2::RIGHT = Vector2(1.f, 0.f);
}

