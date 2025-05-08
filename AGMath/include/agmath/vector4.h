#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <format>
#include <string>

#include "utilities.h"

namespace agm
{
	struct Vector4
	{
		static const Vector4 ZERO;
		static const Vector4 ONE;

		union
		{
			struct
			{
				float x, y, z, w;
			};

			std::array<float, 4> data;
		};

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

		inline constexpr float operator[](size_t index) const
		{
			return data[index];
		}

		inline constexpr float& operator[](size_t index)
		{
			return data[index];
		}

		inline constexpr Vector4 operator-() const
		{
			return Vector4(-x, -y, -z, -w);
		}

		inline constexpr Vector4 operator*(float scalar) const
		{
			return Vector4(x * scalar, y * scalar, z * scalar, w * scalar);
		}

		inline constexpr Vector4 operator/(float scalar) const
		{
			return Vector4(x / scalar, y / scalar, z / scalar, w / scalar);
		}

		inline constexpr Vector4 operator+(const Vector4& other) const
		{
			return Vector4(x + other.x, y + other.y, z + other.z, w + other.w);
		}

		inline constexpr Vector4 operator-(const Vector4& other) const
		{
			return Vector4(x - other.x, y - other.y, z - other.z, w - other.w);
		}

		inline constexpr Vector4 operator*(const Vector4& other) const
		{
			return Vector4(x * other.x, y * other.y, z * other.z, w * other.w);
		}

		inline constexpr Vector4 operator/(const Vector4& other) const
		{
			return Vector4(x / other.x, y / other.y, z / other.z, w / other.w);
		}

		inline constexpr Vector4& operator*=(float scalar)
		{
			x *= scalar;
			y *= scalar;
			z *= scalar;
			w *= scalar;
			return *this;
		}

		inline constexpr Vector4& operator/=(float scalar)
		{
			x /= scalar;
			y /= scalar;
			z /= scalar;
			w /= scalar;
			return *this;
		}

		inline constexpr Vector4& operator+=(const Vector4& other)
		{
			x += other.x;
			y += other.y;
			z += other.z;
			w += other.w;
			return *this;
		}

		inline constexpr Vector4& operator-=(const Vector4& other)
		{
			x -= other.x;
			y -= other.y;
			z -= other.z;
			w -= other.w;
			return *this;
		}

		inline constexpr Vector4& operator*=(const Vector4& other)
		{
			x *= other.x;
			y *= other.y;
			z *= other.z;
			w *= other.w;
			return *this;
		}

		inline constexpr Vector4& operator/=(const Vector4& other)
		{
			x /= other.x;
			y /= other.y;
			z /= other.z;
			w /= other.w;
			return *this;
		}

		inline constexpr bool operator==(const Vector4& other) const
		{
			return x == other.x && y == other.y && z == other.z && w == other.w;
		}

		inline constexpr bool operator!=(const Vector4& other) const
		{
			return !(*this == other);
		}

		friend inline constexpr Vector4 operator*(float scalar, const Vector4& v)
		{
			return Vector4(v.x * scalar, v.y * scalar, v.z * scalar, v.w * scalar);
		}

		static inline float Distance(const Vector4& a, const Vector4& b)
		{
			return (a - b).Length();
		}

		static inline constexpr float Dot(const Vector4& a, const Vector4& b)
		{
			return a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
		}

		static inline constexpr Vector4 Lerp(const Vector4& a, const Vector4& b, float t)
		{
			t = Clamp01(t);
			return Vector4(a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t, a.z + (b.z - a.z) * t, a.w + (b.w - a.w) * t);
		}

		static inline constexpr Vector4 LerpUnclamped(const Vector4& a, const Vector4& b, float t)
		{
			return Vector4(a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t, a.z + (b.z - a.z) * t, a.w + (b.w - a.w) * t);
		}

		static inline constexpr Vector4 Max(const Vector4& a, const Vector4& b)
		{
			return Vector4(agm::Max(a.x, b.x), agm::Max(a.y, b.y), agm::Max(a.z, b.z), agm::Max(a.w, b.w));
		}

		static inline constexpr Vector4 Min(const Vector4& a, const Vector4& b)
		{
			return Vector4(agm::Min(a.x, b.x), agm::Min(a.y, b.y), agm::Min(a.z, b.z), agm::Min(a.w, b.w));
		}

		static inline Vector4 MoveTowards(const Vector4& current, const Vector4& target, float maxDistanceDelta)
		{
			Vector4 delta = target - current;
			float lengthSq = delta.LengthSquared();
			if (agm::IsNearlyZero(lengthSq) || (maxDistanceDelta >= 0.f && lengthSq <= maxDistanceDelta * maxDistanceDelta))
			{
				return target;
			}

			float length = std::sqrt(lengthSq);
			return current + delta / length * maxDistanceDelta;
		}

		static inline constexpr Vector4 Project(const Vector4& a, const Vector4& b)
		{
			return b * (Dot(a, b) / Dot(b, b));
		}

		inline constexpr bool Equals(const Vector4& other, float tolerance = LOOSE_EPSILON) const
		{
			return Abs(x - other.x) <= tolerance && Abs(y - other.y) <= tolerance && Abs(z - other.z) <= tolerance && Abs(w - other.w) <= tolerance;
		}

		inline constexpr bool IsNearlyZero(float tolerance = LOOSE_EPSILON) const
		{
			return Abs(x) <= tolerance && Abs(y) <= tolerance && Abs(z) <= tolerance && Abs(w) <= tolerance;
		}

		inline constexpr bool IsZero() const
		{
			return x == 0.f && y == 0.f && z == 0.f && w == 0.f;
		}

		inline constexpr bool IsNormalized() const
		{
			return Abs(1.f - LengthSquared()) < 0.01f;
		}

		inline float Length() const
		{
			return std::sqrt(x * x + y * y + z * z + w * w);
		}

		inline constexpr float LengthSquared() const
		{
			return x * x + y * y + z * z + w * w;
		}

		inline void Normalize(float tolerance = EPSILON)
		{
			*this = Normalized(tolerance);
		}

		inline Vector4 Normalized(float tolerance = EPSILON) const
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

		inline constexpr void Set(float x, float y, float z, float w)
		{
			this->x = x;
			this->y = y;
			this->z = z;
			this->w = w;
		}

		std::string ToString() const
		{
			return std::format("({:.2f}, {:.2f}, {:.2f}, {:.2f})", x, y, z, w);
		}
	};

	inline const Vector4 Vector4::ZERO = Vector4(0.f, 0.f, 0.f, 0.f);
	inline const Vector4 Vector4::ONE = Vector4(1.f, 1.f, 1.f, 1.f);
}

