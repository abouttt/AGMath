#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <format>
#include <string>

#include "utilities.h"

namespace agm
{
	struct Vector3
	{
		static const Vector3 ZERO;
		static const Vector3 ONE;
		static const Vector3 UP;
		static const Vector3 DOWN;
		static const Vector3 LEFT;
		static const Vector3 RIGHT;
		static const Vector3 FORWARD;
		static const Vector3 BACK;

		union
		{
			struct
			{
				float x, y, z;
			};

			std::array<float, 3> data;
		};

		constexpr Vector3()
			: x(0.f)
			, y(0.f)
			, z(0.f)
		{
		}

		constexpr Vector3(float x, float y, float z)
			: x(x)
			, y(y)
			, z(z)
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

		inline constexpr Vector3 operator-() const
		{
			return Vector3(-x, -y, -z);
		}

		inline constexpr Vector3 operator*(float scalar) const
		{
			return Vector3(x * scalar, y * scalar, z * scalar);
		}

		inline constexpr Vector3 operator/(float scalar) const
		{
			return Vector3(x / scalar, y / scalar, z / scalar);
		}

		inline constexpr Vector3 operator+(const Vector3& other) const
		{
			return Vector3(x + other.x, y + other.y, z + other.z);
		}

		inline constexpr Vector3 operator-(const Vector3& other) const
		{
			return Vector3(x - other.x, y - other.y, z - other.z);
		}

		inline constexpr Vector3 operator*(const Vector3& other) const
		{
			return Vector3(x * other.x, y * other.y, z * other.z);
		}

		inline constexpr Vector3 operator/(const Vector3& other) const
		{
			return Vector3(x / other.x, y / other.y, z / other.z);
		}

		inline constexpr Vector3& operator*=(float scalar)
		{
			x *= scalar;
			y *= scalar;
			z *= scalar;
			return *this;
		}

		inline constexpr Vector3& operator/=(float scalar)
		{
			x /= scalar;
			y /= scalar;
			z /= scalar;
			return *this;
		}

		inline constexpr Vector3& operator+=(const Vector3& other)
		{
			x += other.x;
			y += other.y;
			z += other.z;
			return *this;
		}

		inline constexpr Vector3& operator-=(const Vector3& other)
		{
			x -= other.x;
			y -= other.y;
			z -= other.z;
			return *this;
		}

		inline constexpr Vector3& operator*=(const Vector3& other)
		{
			x *= other.x;
			y *= other.y;
			z *= other.z;
			return *this;
		}

		inline constexpr Vector3& operator/=(const Vector3& other)
		{
			x /= other.x;
			y /= other.y;
			z /= other.z;
			return *this;
		}

		inline constexpr bool operator==(const Vector3& other) const
		{
			return x == other.x && y == other.y && z == other.z;
		}

		inline constexpr bool operator!=(const Vector3& other) const
		{
			return !(*this == other);
		}

		friend inline constexpr Vector3 operator*(float scalar, const Vector3& v)
		{
			return Vector3(v.x * scalar, v.y * scalar, v.z * scalar);
		}

		static inline float Angle(const Vector3& from, const Vector3& to)
		{
			float length = std::sqrt(from.LengthSquared() * to.LengthSquared());
			if (agm::IsNearlyZero(length))
			{
				return 0.f;
			}

			float cosTheta = Clamp(Dot(from, to) / length, -1.f, 1.f);
			return std::acos(cosTheta) * RAD2DEG;
		}

		static inline Vector3 ClampLength(const Vector3& v, float maxLength)
		{
			float lengthSq = v.LengthSquared();
			if (lengthSq > maxLength * maxLength)
			{
				float length = std::sqrt(lengthSq);
				Vector3 norm = v / length;
				return norm * maxLength;
			}

			return v;
		}

		static inline constexpr Vector3 Cross(const Vector3& a, const Vector3& b)
		{
			return Vector3(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x);
		}

		static inline float Distance(const Vector3& a, const Vector3& b)
		{
			return (a - b).Length();
		}

		static inline constexpr float Dot(const Vector3& a, const Vector3& b)
		{
			return a.x * b.x + a.y * b.y + a.z * b.z;
		}

		static inline constexpr Vector3 Lerp(const Vector3& a, const Vector3& b, float t)
		{
			t = Clamp01(t);
			return Vector3(a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t, a.z + (b.z - a.z) * t);
		}

		static inline constexpr Vector3 LerpUnclamped(const Vector3& a, const Vector3& b, float t)
		{
			return Vector3(a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t, a.z + (b.z - a.z) * t);
		}

		static inline constexpr Vector3 Max(const Vector3& a, const Vector3& b)
		{
			return Vector3(agm::Max(a.x, b.x), agm::Max(a.y, b.y), agm::Max(a.z, b.z));
		}

		static inline constexpr Vector3 Min(const Vector3& a, const Vector3& b)
		{
			return Vector3(agm::Min(a.x, b.x), agm::Min(a.y, b.y), agm::Min(a.z, b.z));
		}

		static inline Vector3 MoveTowards(const Vector3& current, const Vector3& target, float maxDistanceDelta)
		{
			Vector3 delta = target - current;
			float lengthSq = delta.LengthSquared();
			if (agm::IsNearlyZero(lengthSq) || (maxDistanceDelta >= 0.f && lengthSq <= maxDistanceDelta * maxDistanceDelta))
			{
				return target;
			}

			float length = std::sqrt(lengthSq);
			return current + delta / length * maxDistanceDelta;
		}

		static inline void OrthoNormalize(Vector3& inoutNormal, Vector3& inoutTangent)
		{
			inoutNormal.Normalize();
			inoutTangent = ProjectOnPlane(inoutTangent, inoutNormal);
			inoutTangent.Normalize();
		}

		static inline void OrthoNormalize(Vector3& inoutNormal, Vector3& inoutTangent, Vector3& inoutBinormal)
		{
			OrthoNormalize(inoutNormal, inoutTangent);
			inoutBinormal = Cross(inoutNormal, inoutTangent);
			inoutBinormal.Normalize();
		}

		static inline constexpr Vector3 Project(const Vector3& v, const Vector3& normal)
		{
			float normalLengthSq = Dot(normal, normal);
			if (agm::IsNearlyZero(normalLengthSq))
			{
				return Vector3::ZERO;
			}

			return normal * Dot(v, normal) / normalLengthSq;
		}

		static inline constexpr Vector3 ProjectOnPlane(const Vector3& v, const Vector3& planeNormal)
		{
			float normalLengthSq = Dot(planeNormal, planeNormal);
			if (agm::IsNearlyZero(normalLengthSq))
			{
				return v;
			}

			return v - planeNormal * Dot(v, planeNormal) / normalLengthSq;
		}

		static inline constexpr Vector3 Reflect(const Vector3& inDirection, const Vector3& inNormal)
		{
			return (-2.f * Dot(inDirection, inNormal)) * inNormal + inDirection;
		}

		static inline Vector3 RotateAroundAxis(const Vector3& vector, const Vector3& axis, float angle)
		{
			Vector3 unitAxis = axis.Normalized();
			float radians = angle * DEG2RAD;
			float cosTheta = std::cos(radians);
			float sinTheta = std::sin(radians);
			return vector * cosTheta + Cross(unitAxis, vector) * sinTheta + unitAxis * Dot(unitAxis, vector) * (1.f - cosTheta);
		}

		static inline Vector3 RotateTowards(const Vector3& current, const Vector3& target, float maxRadiansDelta, float maxLengthDelta)
		{
			const float currentLength = current.Length();
			const float targetLength = target.Length();

			if (currentLength < EPSILON || targetLength < EPSILON)
			{
				return MoveTowards(current, target, maxLengthDelta);
			}

			Vector3 currentDirection = current / currentLength;
			Vector3 targetDirection = target / targetLength;

			float cosTheta = Clamp(Dot(currentDirection, targetDirection), -1.f, 1.f);
			float angle = std::acos(cosTheta);

			if (angle < EPSILON)
			{
				float newLength = agm::MoveTowards(currentLength, targetLength, maxLengthDelta);
				return currentDirection * newLength;
			}

			float t = agm::Min(1.f, maxRadiansDelta / angle);
			Vector3 newDirection = SlerpUnclamped(currentDirection, targetDirection, t);
			float newLength = agm::MoveTowards(currentLength, targetLength, maxLengthDelta);

			return newDirection * newLength;
		}

		static inline float SignedAngle(const Vector3& from, const Vector3& to, const Vector3& axis)
		{
			return Angle(from, to) * Sign(Dot(axis, Cross(from, to)));
		}

		static inline Vector3 Slerp(const Vector3& a, const Vector3& b, float t)
		{
			t = Clamp01(t);
			return SlerpUnclamped(a, b, t);
		}

		static inline Vector3 SlerpUnclamped(const Vector3& a, const Vector3& b, float t)
		{
			float lengthA = a.Length();
			float lengthB = b.Length();

			if (lengthA < EPSILON || lengthB < EPSILON)
			{
				return LerpUnclamped(a, b, t);
			}

			Vector3 normA = a / lengthA;
			Vector3 normB = b / lengthB;

			float dot = Clamp(Dot(normA, normB), -1.f, 1.f);
			float theta = std::acos(dot) * t;

			Vector3 relative = (normB - normA * dot).Normalized();
			Vector3 direction = normA * std::cos(theta) + relative * std::sin(theta);
			float length = agm::LerpUnclamped(lengthA, lengthB, t);
			return direction * length;
		}

		inline constexpr bool Equals(const Vector3& other, float tolerance = LOOSE_EPSILON) const
		{
			return Abs(x - other.x) <= tolerance && Abs(y - other.y) <= tolerance && Abs(z - other.z) <= tolerance;
		}

		inline constexpr bool IsNearlyZero(float tolerance = LOOSE_EPSILON) const
		{
			return Abs(x) <= tolerance && Abs(y) <= tolerance && Abs(z) <= tolerance;
		}

		inline constexpr bool IsZero() const
		{
			return x == 0.f && y == 0.f && z == 0.f;
		}

		inline constexpr bool IsNormalized() const
		{
			return Abs(1.f - LengthSquared()) < 0.01f;
		}

		inline float Length() const
		{
			return std::sqrt(x * x + y * y + z * z);
		}

		inline constexpr float LengthSquared() const
		{
			return x * x + y * y + z * z;
		}

		inline void Normalize(float tolerance = EPSILON)
		{
			*this = Normalized(tolerance);
		}

		inline Vector3 Normalized(float tolerance = EPSILON) const
		{
			float lengthSq = LengthSquared();
			if (lengthSq > tolerance)
			{
				float invLength = 1.f / std::sqrt(lengthSq);
				return *this * invLength;
			}
			else
			{
				return Vector3::ZERO;
			}
		}

		inline constexpr void Set(float x, float y, float z)
		{
			this->x = x;
			this->y = y;
			this->z = z;
		}

		std::string ToString() const
		{
			return std::format("({:.2f}, {:.2f}, {:.2f})", x, y, z);
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

