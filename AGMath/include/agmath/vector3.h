#pragma once

#include <cmath>
#include <cstdint>
#include <format>
#include <stdexcept>
#include <string>

#include "utilities.h"

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

		constexpr Vector3(float x, float y, float z)
			: x(x)
			, y(y)
			, z(z)
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
			default: throw std::out_of_range("Invalid Vector3 index!");
			}
		}

		constexpr float& operator[](int32_t index)
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
			return Vector3(v.x * scalar, v.y * scalar, v.z * scalar);
		}

	public:

		float Length() const
		{
			return std::sqrt(x * x + y * y + z * z);
		}

		constexpr float LengthSquared() const
		{
			return x * x + y * y + z * z;
		}

		void Normalize(float tolerance = EPSILON)
		{
			*this = GetNormalized(tolerance);
		}

		Vector3 GetNormalized(float tolerance = EPSILON) const
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

		constexpr bool IsNormalized() const
		{
			return Abs(1.f - LengthSquared()) < THRESH_VECTOR_NORMALIZED;
		}

		Vector3 RotateAngleAxis(float angle, const Vector3& axis)
		{
			float radians = angle * DEG2RAD;
			float cosTheta = std::cos(radians);
			float sinTheta = std::sin(radians);

			float xx = axis.x * axis.x;
			float yy = axis.y * axis.y;
			float zz = axis.z * axis.z;

			float xy = axis.x * axis.y;
			float yz = axis.y * axis.z;
			float zx = axis.z * axis.x;

			float xs = axis.x * sinTheta;
			float ys = axis.y * sinTheta;
			float zs = axis.z * sinTheta;

			float omc = 1.f - cosTheta;

			return Vector3(
				(omc * xx + cosTheta) * x + (omc * xy - zs) * y + (omc * zx + ys) * z,
				(omc * xy + zs) * x + (omc * yy + cosTheta) * y + (omc * yz - xs) * z,
				(omc * zx - ys) * x + (omc * yz + xs) * y + (omc * zz + cosTheta) * z
			);
		}

		constexpr Vector3 GetAbs() const
		{
			return Vector3(Abs(x), Abs(y), Abs(z));
		}

		constexpr float GetMax() const
		{
			return Max3(x, y, z);
		}

		constexpr float GetAbsMax() const
		{
			return Max3(Abs(x), Abs(y), Abs(z));
		}

		constexpr float GetMin() const
		{
			return Min3(x, y, z);
		}

		constexpr float GetAbsMin() const
		{
			return Min3(Abs(x), Abs(y), Abs(z));
		}

		constexpr bool IsNearlyZero(float tolerance = LOOSE_EPSILON) const
		{
			return Abs(x) <= tolerance && Abs(y) <= tolerance && Abs(z) <= tolerance;
		}

		constexpr bool IsZero() const
		{
			return x == 0.f && y == 0.f && z == 0.f;
		}

		constexpr void Set(float x, float y, float z)
		{
			this->x = x;
			this->y = y;
			this->z = z;
		}

		constexpr bool Equals(const Vector3& other, float tolerance = LOOSE_EPSILON) const
		{
			return Abs(x - other.x) <= tolerance && Abs(y - other.y) <= tolerance && Abs(z - other.z) <= tolerance;
		}

		std::string ToString() const
		{
			return std::format("({:.2f}, {:.2f}, {:.2f})", x, y, z);
		}

	public:

		static float Angle(const Vector3& from, const Vector3& to)
		{
			float length = std::sqrt(from.LengthSquared() * to.LengthSquared());
			if (agm::IsNearlyZero(length))
			{
				return 0.f;
			}

			float cosTheta = agm::Clamp(Dot(from, to) / length, -1.f, 1.f);
			return std::acos(cosTheta) * RAD2DEG;
		}

		static Vector3 Clamp(const Vector3& v, const Vector3& min, const Vector3& max)
		{
			return Vector3(agm::Clamp(v.x, min.x, max.x), agm::Clamp(v.y, min.y, max.y), agm::Clamp(v.z, min.z, max.z));
		}

		static Vector3 ClampLength(const Vector3& v, float maxLength)
		{
			float lengthSq = v.LengthSquared();
			if (lengthSq > maxLength * maxLength)
			{
				float length = std::sqrt(lengthSq);
				Vector3 unit = v / length;
				return unit * maxLength;
			}

			return v;
		}

		static constexpr Vector3 Cross(const Vector3& a, const Vector3& b)
		{
			return Vector3(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x);
		}

		static float Distance(const Vector3& a, const Vector3& b)
		{
			return (a - b).Length();
		}

		static constexpr float Dot(const Vector3& a, const Vector3& b)
		{
			return a.x * b.x + a.y * b.y + a.z * b.z;
		}

		static constexpr Vector3 Lerp(const Vector3& a, const Vector3& b, float t)
		{
			t = Clamp01(t);
			return Vector3(a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t, a.z + (b.z - a.z) * t);
		}

		static constexpr Vector3 LerpUnclamped(const Vector3& a, const Vector3& b, float t)
		{
			return Vector3(a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t, a.z + (b.z - a.z) * t);
		}

		static constexpr Vector3 Max(const Vector3& a, const Vector3& b)
		{
			return Vector3(agm::Max(a.x, b.x), agm::Max(a.y, b.y), agm::Max(a.z, b.z));
		}

		static constexpr Vector3 Min(const Vector3& a, const Vector3& b)
		{
			return Vector3(agm::Min(a.x, b.x), agm::Min(a.y, b.y), agm::Min(a.z, b.z));
		}

		static Vector3 MoveTowards(const Vector3& current, const Vector3& target, float maxDistanceDelta)
		{
			Vector3 direction = target - current;
			float lengthSq = direction.LengthSquared();
			if (agm::IsNearlyZero(lengthSq) || (maxDistanceDelta >= 0.f && lengthSq <= maxDistanceDelta * maxDistanceDelta))
			{
				return target;
			}

			float length = std::sqrt(lengthSq);
			return current + direction / length * maxDistanceDelta;
		}

		static void OrthoNormalize(Vector3& inoutNormal, Vector3& inoutTangent)
		{
			inoutNormal.Normalize();
			inoutTangent = ProjectOnPlane(inoutTangent, inoutNormal);
			inoutTangent.Normalize();
		}

		static void OrthoNormalize(Vector3& inoutNormal, Vector3& inoutTangent, Vector3& inoutBinormal)
		{
			OrthoNormalize(inoutNormal, inoutTangent);
			inoutBinormal = Cross(inoutNormal, inoutTangent);
			inoutBinormal.Normalize();
		}

		static constexpr Vector3 Project(const Vector3& v, const Vector3& onNormal)
		{
			float normalLengthSq = Dot(onNormal, onNormal);
			if (agm::IsNearlyZero(normalLengthSq))
			{
				return Vector3::ZERO;
			}

			return onNormal * Dot(v, onNormal) / normalLengthSq;
		}

		static constexpr Vector3 ProjectOnPlane(const Vector3& v, const Vector3& planeNormal)
		{
			float normalLengthSq = Dot(planeNormal, planeNormal);
			if (agm::IsNearlyZero(normalLengthSq))
			{
				return v;
			}

			return v - planeNormal * Dot(v, planeNormal) / normalLengthSq;
		}

		static constexpr Vector3 Reflect(const Vector3& inDirection, const Vector3& inNormal)
		{
			return (-2.f * Dot(inDirection, inNormal)) * inNormal + inDirection;
		}

		static Vector3 RotateTowards(const Vector3& current, const Vector3& target, float maxRadiansDelta, float maxLengthDelta)
		{
			const float currentLength = current.Length();
			const float targetLength = target.Length();

			if (currentLength < EPSILON || targetLength < EPSILON)
			{
				return MoveTowards(current, target, maxLengthDelta);
			}

			Vector3 currentDirection = current / currentLength;
			Vector3 targetDirection = target / targetLength;

			float cosTheta = agm::Clamp(Dot(currentDirection, targetDirection), -1.f, 1.f);
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

		static float SignedAngle(const Vector3& from, const Vector3& to, const Vector3& axis)
		{
			return Angle(from, to) * Sign(Dot(axis, Cross(from, to)));
		}

		static Vector3 Slerp(const Vector3& a, const Vector3& b, float t)
		{
			t = Clamp01(t);
			return SlerpUnclamped(a, b, t);
		}

		static Vector3 SlerpUnclamped(const Vector3& a, const Vector3& b, float t)
		{
			float lengthA = a.Length();
			float lengthB = b.Length();

			if (lengthA < EPSILON || lengthB < EPSILON)
			{
				return LerpUnclamped(a, b, t);
			}

			Vector3 unitA = a / lengthA;
			Vector3 unitB = b / lengthB;

			float dot = agm::Clamp(Dot(unitA, unitB), -1.f, 1.f);
			float theta = std::acos(dot) * t;

			Vector3 relative = (unitB - unitA * dot).GetNormalized();
			Vector3 direction = unitA * std::cos(theta) + relative * std::sin(theta);
			float length = agm::LerpUnclamped(lengthA, lengthB, t);

			return direction * length;
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

