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
			return Vector3(v.x * scalar, v.y * scalar, v.z * scalar);
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

		void Normalize(float tolerance = EPSILON)
		{
			*this = GetNormalized(tolerance);
		}

		Vector3 GetNormalized(float tolerance = EPSILON) const
		{
			float lengthSq = LengthSquared();
			if (lengthSq > tolerance)
			{
				return *this * agm::InvSqrt(lengthSq);;
			}
			else
			{
				return Vector3::ZERO;
			}
		}

		constexpr bool IsNormalized(float threshold = agm::VECTOR_NORMALIZED_THRESHOLD) const
		{
			return agm::Abs(1.f - LengthSquared()) < threshold;
		}

		constexpr Vector3 GetAbs() const
		{
			return Vector3(agm::Abs(x), agm::Abs(y), agm::Abs(z));
		}

		constexpr float GetMax() const
		{
			return agm::Max3(x, y, z);
		}

		constexpr float GetAbsMax() const
		{
			return agm::Max3(agm::Abs(x), agm::Abs(y), agm::Abs(z));
		}

		constexpr float GetMin() const
		{
			return agm::Min3(x, y, z);
		}

		constexpr float GetAbsMin() const
		{
			return agm::Min3(agm::Abs(x), agm::Abs(y), agm::Abs(z));
		}

		constexpr bool IsNearlyZero(float tolerance = EPSILON) const
		{
			return agm::IsNearlyZero(x, tolerance) && agm::IsNearlyZero(y, tolerance) && agm::IsNearlyZero(z, tolerance);
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

		constexpr bool Equals(const Vector3& other, float tolerance = EPSILON) const
		{
			return
				agm::IsNearlyEqual(x, other.x, tolerance) &&
				agm::IsNearlyEqual(y, other.y, tolerance) &&
				agm::IsNearlyEqual(z, other.z, tolerance);
		}

		std::string ToString() const
		{
			return std::format("({:.2f}, {:.2f}, {:.2f})", x, y, z);
		}

	public:

		static float Angle(const Vector3& from, const Vector3& to)
		{
			float denominatorSq = from.LengthSquared() * to.LengthSquared();
			if (agm::IsNearlyZero(denominatorSq))
			{
				return 0.f;
			}

			float cosTheta = Dot(from, to) / agm::Sqrt(denominatorSq);
			return std::acos(agm::Clamp(cosTheta, -1.f, 1.f)) * agm::RAD2DEG;
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

			float lengthSq = v.LengthSquared();
			if (lengthSq > maxLength * maxLength && !agm::IsNearlyZero(lengthSq))
			{
				return v * (maxLength / agm::Sqrt(lengthSq));
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
			Vector3 absV = v.GetAbs();
			Vector3 other;
			if (absV.x < absV.y)
			{
				other = (absV.x < absV.z) ? Vector3::RIGHT : Vector3::FORWARD;
			}
			else
			{
				other = (absV.y < absV.z) ? Vector3::UP : Vector3::FORWARD;
			}

			if (!agm::IsNearlyZero(v.x) || !agm::IsNearlyZero(v.y))
			{
				return Cross(v, Vector3::FORWARD).GetNormalized();
			}
			else
			{
				return Cross(v, Vector3::RIGHT).GetNormalized();
			}
		}

		static constexpr Vector3 Lerp(const Vector3& a, const Vector3& b, float t)
		{
			float clampedT = agm::Clamp01(t);
			return Vector3(
				a.x + (b.x - a.x) * clampedT,
				a.y + (b.y - a.y) * clampedT,
				a.z + (b.z - a.z) * clampedT
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

		static Vector3 MoveTowards(const Vector3& current, const Vector3& target, float maxDistanceDelta)
		{
			float effMaxDistanceDelta = maxDistanceDelta < 0.f ? 0.f : maxDistanceDelta;
			Vector3 delta = target - current;
			float distSq = delta.LengthSquared();
			if (agm::IsNearlyZero(distSq) || distSq <= effMaxDistanceDelta * effMaxDistanceDelta)
			{
				return target;
			}

			return current + delta * (effMaxDistanceDelta / agm::Sqrt(distSq));
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
			float normalLengthSq = Dot(onNormal, onNormal);
			if (agm::IsNearlyZero(normalLengthSq))
			{
				return Vector3::ZERO;
			}

			return onNormal * (Dot(v, onNormal) / normalLengthSq);
		}

		static constexpr Vector3 ProjectOnPlane(const Vector3& v, const Vector3& planeNormal)
		{
			float normalLengthSq = Dot(planeNormal, planeNormal);
			if (agm::IsNearlyZero(normalLengthSq))
			{
				return v;
			}

			return v - planeNormal * (Dot(v, planeNormal) / normalLengthSq);
		}

		static constexpr Vector3 Reflect(const Vector3& inDirection, const Vector3& inNormal)
		{
			float normalLengthSq = Dot(inNormal, inNormal);
			if (agm::IsNearlyZero(normalLengthSq))
			{
				return inDirection;
			}

			return inDirection - (2.f * Dot(inDirection, inNormal) / normalLengthSq) * inNormal;
		}

		static Vector3 RotateTowards(const Vector3& current, const Vector3& target, float maxRadiansDelta, float maxLengthDelta)
		{
			float effMaxRadiansDelta = maxRadiansDelta < 0.f ? 0.f : maxRadiansDelta;
			float effMaxLengthDelta = maxLengthDelta;

			float currentLength = current.Length();
			float targetLength = target.Length();

			if (agm::IsNearlyZero(currentLength) || agm::IsNearlyZero(targetLength))
			{
				if (agm::IsNearlyZero(currentLength) && agm::IsNearlyZero(targetLength))
				{
					return target;
				}

				Vector3 nonZeroDir = agm::IsNearlyZero(currentLength) ? target.GetNormalized() : current.GetNormalized();
				float newMag = agm::MoveTowards(currentLength, targetLength, effMaxLengthDelta);
				return nonZeroDir * newMag;
			}

			Vector3 currentDirection = current / currentLength;
			Vector3 targetDirection = target / targetLength;

			float cosTheta = agm::Clamp(Dot(currentDirection, targetDirection), -1.f, 1.f);
			float angle = std::acos(cosTheta);

			float finalLength = agm::MoveTowards(currentLength, targetLength, effMaxLengthDelta);

			if (agm::IsNearlyZero(angle) || agm::IsNearlyZero(effMaxRadiansDelta))
			{
				return currentDirection * finalLength;
			}

			float t = agm::Min(1.f, effMaxRadiansDelta / angle);

			Vector3 rotationAxis = Cross(currentDirection, targetDirection);
			if (rotationAxis.IsNearlyZero())
			{
				if (IsNearlyEqual(cosTheta, -1.f))
				{
					Vector3 arbitraryAxis = FindOrthogonal(currentDirection);
					if (arbitraryAxis.IsNearlyZero())
					{
						return currentDirection * finalLength;
					}

					float rotAngle = t * agm::PI;
					Vector3 rotatedDirection = currentDirection * std::cos(rotAngle) + Cross(arbitraryAxis, currentDirection) * std::sin(rotAngle);
					return rotatedDirection.GetNormalized() * finalLength;
				}

				return currentDirection * finalLength;
			}

			rotationAxis.Normalize();

			Vector3 relative = (targetDirection - currentDirection * cosTheta);
			if (relative.IsNearlyZero())
			{
				return currentDirection * finalLength;
			}
			relative.Normalize();

			Vector3 direction = currentDirection * std::cos(t * angle) + relative * std::sin(t * angle);
			return direction * finalLength;
		}

		static float SignedAngle(const Vector3& from, const Vector3& to, const Vector3& axis)
		{
			float unsignedAngle = Angle(from, to);
			float sign = agm::Sign(Dot(axis, Cross(from, to)));
			return unsignedAngle * sign;
		}

		static Vector3 Slerp(const Vector3& a, const Vector3& b, float t)
		{
			float clampedT = agm::Clamp01(t);

			float lengthA = a.Length();
			float lengthB = b.Length();

			float finalLength = agm::Lerp(lengthA, lengthB, clampedT);

			if (agm::IsNearlyZero(lengthA) || agm::IsNearlyZero(lengthB))
			{
				return Lerp(a, b, clampedT);
			}

			Vector3 unitA = a / lengthA;
			Vector3 unitB = b / lengthB;

			float dot = agm::Clamp(Dot(unitA, unitB), -1.f, 1.f);

			if (agm::IsNearlyEqual(agm::Abs(dot), 1.f))
			{
				return unitA * finalLength;
			}

			float theta0 = std::acos(dot);
			float theta = theta0 * clampedT;

			Vector3 relative = (unitB - unitA * dot).GetNormalized();
			if (relative.IsNearlyZero())
			{
				return Lerp(a, b, clampedT);
			}

			Vector3 direction = unitA * std::cos(theta) + relative * std::sin(theta);
			return direction * finalLength;
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

