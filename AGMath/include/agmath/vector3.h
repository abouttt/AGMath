#pragma once

#include <cmath>
#include <cstdint>
#include <format>
#include <stdexcept>
#include <string>

#include "utilities.h"
#include "vector2.h"

namespace agm
{
	struct Vector3
	{
	public:

		float x;
		float y;
		float z;

	public:

		static const Vector3 ZERO;
		static const Vector3 ONE;
		static const Vector3 UP;
		static const Vector3 DOWN;
		static const Vector3 LEFT;
		static const Vector3 RIGHT;
		static const Vector3 FORWARD;
		static const Vector3 BACK;

	public:

		constexpr Vector3()
			: x(0.f)
			, y(0.f)
			, z(0.f)
		{
		}

		constexpr Vector3(float fill)
			: x(fill)
			, y(fill)
			, z(fill)
		{
		}

		constexpr Vector3(float x, float y, float z)
			: x(x)
			, y(y)
			, z(z)
		{
		}

		constexpr explicit Vector3(const Vector2& v2, float z = 0.f)
			: x(v2.x)
			, y(v2.y)
			, z(z)
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
			default: throw std::out_of_range("Invalid Vector3 index!");
			}
		}

		float& operator[](int32_t index)
		{
			switch (index)
			{
			case 0: return x;
			case 1: return y;
			case 2: return z;
			default: throw std::out_of_range("Invalid Vector3 index!");
			}
		}

		constexpr Vector3 operator-() const
		{
			return Vector3(-x, -y, -z);
		}

		constexpr Vector3 operator*(float scalar) const
		{
			return Vector3(x * scalar, y * scalar, z * scalar);
		}

		constexpr Vector3 operator/(float scalar) const
		{
			return Vector3(x / scalar, y / scalar, z / scalar);
		}

		constexpr Vector3 operator+(const Vector3& other) const
		{
			return Vector3(x + other.x, y + other.y, z + other.z);
		}

		constexpr Vector3 operator-(const Vector3& other) const
		{
			return Vector3(x - other.x, y - other.y, z - other.z);
		}

		constexpr Vector3 operator*(const Vector3& other) const
		{
			return Vector3(x * other.x, y * other.y, z * other.z);
		}

		constexpr Vector3 operator/(const Vector3& other) const
		{
			return Vector3(x / other.x, y / other.y, z / other.z);
		}

		constexpr Vector3& operator*=(float scalar)
		{
			x *= scalar;
			y *= scalar;
			z *= scalar;
			return *this;
		}

		constexpr Vector3& operator/=(float scalar)
		{
			x /= scalar;
			y /= scalar;
			z /= scalar;
			return *this;
		}

		constexpr Vector3& operator+=(const Vector3& other)
		{
			x += other.x;
			y += other.y;
			z += other.z;
			return *this;
		}

		constexpr Vector3& operator-=(const Vector3& other)
		{
			x -= other.x;
			y -= other.y;
			z -= other.z;
			return *this;
		}

		constexpr Vector3& operator*=(const Vector3& other)
		{
			x *= other.x;
			y *= other.y;
			z *= other.z;
			return *this;
		}

		constexpr Vector3& operator/=(const Vector3& other)
		{
			x /= other.x;
			y /= other.y;
			z /= other.z;
			return *this;
		}

		constexpr bool operator==(const Vector3& other) const
		{
			return x == other.x && y == other.y && z == other.z;
		}

		constexpr bool operator!=(const Vector3& other) const
		{
			return !(*this == other);
		}

		friend constexpr Vector3 operator*(float scalar, const Vector3& v)
		{
			return v * scalar;
		}

	public:

		float Length() const
		{
			return agm::Sqrt(x * x + y * y + z * z);
		}

		constexpr float LengthSquared() const
		{
			return x * x + y * y + z * z;
		}

		void Normalize(float tolerance = agm::EPSILON_SQ_LENGTH)
		{
			*this = GetNormalized(tolerance);
		}

		Vector3 GetNormalized(float tolerance = agm::EPSILON_SQ_LENGTH) const
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
				   agm::Abs(z) < tolerance;
		}

		constexpr bool IsZero() const
		{
			return x == 0.f && y == 0.f && z == 0.f;
		}

		constexpr Vector3 GetAbs() const
		{
			return Vector3(agm::Abs(x), agm::Abs(y), agm::Abs(z));
		}

		constexpr float GetMax() const
		{
			return agm::Max3(x, y, z);
		}

		constexpr float GetMin() const
		{
			return agm::Min3(x, y, z);
		}

		constexpr void Set(float x, float y, float z)
		{
			this->x = x;
			this->y = y;
			this->z = z;
		}

		constexpr bool Equals(const Vector3& other, float tolerance = agm::EPSILON) const
		{
			return agm::Abs(x - other.x) < tolerance && 
				   agm::Abs(y - other.y) < tolerance &&
				   agm::Abs(z - other.z) < tolerance;
		}

		std::string ToString() const
		{
			return std::format("({:.3f}, {:.3f}, {:.3f})", x, y, z);
		}

		constexpr Vector2 ToVector2() const
		{
			return Vector2(x, y);
		}

	public:

		static float Angle(const Vector3& a, const Vector3& b)
		{
			float denominatorSq = a.LengthSquared() * b.LengthSquared();
			if (agm::IsNearlyZero(denominatorSq, agm::EPSILON_SQ_LENGTH))
			{
				return 0.f;
			}

			float cosTheta = Dot(a, b) / agm::Sqrt(denominatorSq);
			return agm::Acos(agm::Clamp(cosTheta, -1.f, 1.f)) * agm::RAD2DEG;
		}

		static constexpr Vector3 Clamp(const Vector3& v, const Vector3& min, const Vector3& max)
		{
			return Vector3(
				agm::Clamp(v.x, min.x, max.x),
				agm::Clamp(v.y, min.y, max.y),
				agm::Clamp(v.z, min.z, max.z)
			);
		}

		static Vector3 ClampLength(const Vector3& v, float maxLength)
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

		static constexpr Vector3 Cross(const Vector3& a, const Vector3& b)
		{
			return Vector3(
				a.y * b.z - a.z * b.y,
				a.z * b.x - a.x * b.z,
				a.x * b.y - a.y * b.x
			);
		}

		static float Distance(const Vector3& a, const Vector3& b)
		{
			return (a - b).Length();
		}

		static constexpr float DistanceSquared(const Vector3& a, const Vector3& b)
		{
			return (a - b).LengthSquared();
		}

		static constexpr float Dot(const Vector3& a, const Vector3& b)
		{
			return a.x * b.x + a.y * b.y + a.z * b.z;
		}

		static Vector3 FindOrthogonal(const Vector3& v)
		{
			if (v.IsNearlyZero(agm::EPSILON_SQ_LENGTH))
			{
				return Vector3::ZERO;
			}

			if (agm::Abs(v.x) > agm::Abs(v.y))
			{
				return Vector3(-v.z, 0.f, v.x).GetNormalized();
			}
			else
			{
				return Vector3(0.f, -v.z, v.y).GetNormalized();
			}
		}

		static constexpr Vector3 Lerp(const Vector3& a, const Vector3& b, float t)
		{
			t = agm::Clamp01(t);
			return Vector3(
				a.x + (b.x - a.x) * t,
				a.y + (b.y - a.y) * t,
				a.z + (b.z - a.z) * t
			);
		}

		static constexpr Vector3 Max(const Vector3& a, const Vector3& b)
		{
			return Vector3(agm::Max(a.x, b.x), agm::Max(a.y, b.y), agm::Max(a.z, b.z));
		}

		static constexpr Vector3 Min(const Vector3& a, const Vector3& b)
		{
			return Vector3(agm::Min(a.x, b.x), agm::Min(a.y, b.y), agm::Min(a.z, b.z));
		}

		static Vector3 MoveTowards(const Vector3& current, const Vector3& target, float maxDelta)
		{
			if (maxDelta < 0.f)
			{
				maxDelta = 0.f;
			}

			Vector3 delta = target - current;
			float distSq = delta.LengthSquared();
			if (distSq <= maxDelta * maxDelta || agm::IsNearlyZero(distSq, agm::EPSILON_SQ_LENGTH))
			{
				return target;
			}

			return current + delta * (maxDelta / agm::Sqrt(distSq));
		}

		static void OrthoNormalize(Vector3& normal, Vector3& tangent)
		{
			normal.Normalize();
			tangent = ProjectOnPlane(tangent, normal);
			tangent.Normalize();
		}

		static void OrthoNormalize(Vector3& normal, Vector3& tangent, Vector3& binormal)
		{
			OrthoNormalize(normal, tangent);
			binormal = Cross(normal, tangent);
			binormal.Normalize();
		}

		static constexpr Vector3 Project(const Vector3& v, const Vector3& onNormal)
		{
			float normalLenSq = Dot(onNormal, onNormal);
			if (agm::IsNearlyZero(normalLenSq, agm::EPSILON_SQ_LENGTH))
			{
				return ZERO;
			}

			return onNormal * (Dot(v, onNormal) / normalLenSq);
		}

		static constexpr Vector3 ProjectOnPlane(const Vector3& v, const Vector3& planeNormal)
		{
			return v - Project(v, planeNormal);
		}

		static constexpr Vector3 Reflect(const Vector3& inDirection, const Vector3& inNormal)
		{
			return inDirection - Project(inDirection, inNormal) * 2.f;
		}

		static float SignedAngle(const Vector3& a, const Vector3& b, const Vector3& axis)
		{
			float angle = Angle(a, b);
			float sign = Dot(axis, Cross(a, b));
			return angle * agm::Sign(sign);
		}
	};

	inline const Vector3 Vector3::ZERO = Vector3(0.f, 0.f, 0.f);
	inline const Vector3 Vector3::ONE = Vector3(1.f, 1.f, 1.f);
	inline const Vector3 Vector3::UP = Vector3(0.f, 1.f, 0.f);
	inline const Vector3 Vector3::DOWN = Vector3(0.f, -1.f, 0.f);
	inline const Vector3 Vector3::LEFT = Vector3(-1.f, 0.f, 0.f);
	inline const Vector3 Vector3::RIGHT = Vector3(1.f, 0.f, 0.f);
	inline const Vector3 Vector3::FORWARD = Vector3(0.f, 0.f, 1.f);
	inline const Vector3 Vector3::BACK = Vector3(0.f, 0.f, -1.f);
}

