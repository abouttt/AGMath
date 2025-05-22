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
	struct Quaternion
	{
	public:

		float x;
		float y;
		float z;
		float w;

	public:

		static const Quaternion IDENTITY;

	public:

		constexpr Quaternion()
			: x(0.f)
			, y(0.f)
			, z(0.f)
			, w(1.f)
		{
		}

		constexpr Quaternion(float x, float y, float z, float w)
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
			default: throw std::out_of_range("Invalid Quaternion index!");
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
			default: throw std::out_of_range("Invalid Quaternion index!");
			}
		}

		constexpr Quaternion operator-() const
		{
			return Quaternion(-x, -y, -z, -w);
		}

		constexpr Quaternion operator*(float scalar) const
		{
			return Quaternion(x * scalar, y * scalar, z * scalar, w * scalar);
		}

		constexpr Quaternion operator/(float scalar) const
		{
			return Quaternion(x / scalar, y / scalar, z / scalar, w / scalar);
		}

		constexpr Quaternion operator+(const Quaternion& other) const
		{
			return Quaternion(x + other.x, y + other.y, z + other.z, w + other.w);
		}

		constexpr Quaternion operator-(const Quaternion& other) const
		{
			return Quaternion(x - other.x, y - other.y, z - other.z, w - other.w);
		}

		constexpr Quaternion operator*(const Quaternion& other) const
		{
			return Quaternion(
				w * other.x + x * other.w + y * other.z - z * other.y,
				w * other.y - x * other.z + y * other.w + z * other.x,
				w * other.z + x * other.y - y * other.x + z * other.w,
				w * other.w - x * other.x - y * other.y - z * other.z
			);
		}

		constexpr Quaternion& operator*=(float scalar)
		{
			x *= scalar;
			y *= scalar;
			z *= scalar;
			w *= scalar;
			return *this;
		}

		constexpr Quaternion& operator/=(float scalar)
		{
			x /= scalar;
			y /= scalar;
			z /= scalar;
			w /= scalar;
			return *this;
		}

		constexpr Quaternion& operator+=(const Quaternion& other)
		{
			x += other.x;
			y += other.y;
			z += other.z;
			w += other.w;
			return *this;
		}

		constexpr Quaternion& operator-=(const Quaternion& other)
		{
			x -= other.x;
			y -= other.y;
			z -= other.z;
			w -= other.w;
			return *this;
		}

		constexpr Quaternion& operator*=(const Quaternion& other)
		{
			float newX = w * other.x + x * other.w + y * other.z - z * other.y;
			float newY = w * other.y - x * other.z + y * other.w + z * other.x;
			float newZ = w * other.z + x * other.y - y * other.x + z * other.w;
			float newW = w * other.w - x * other.x - y * other.y - z * other.z;

			x = newX;
			y = newY;
			z = newZ;
			w = newW;

			return *this;
		}

		constexpr bool operator==(const Quaternion& other) const
		{
			return x == other.x && y == other.y && z == other.z && w == other.w;
		}

		constexpr bool operator!=(const Quaternion& other) const
		{
			return !(*this == other);
		}

	public:

		float Length() const
		{
			return std::sqrt(x * x + y * y + z * z + w * w);
		}

		constexpr float LengthSquared() const
		{
			return x * x + y * y + z * z + w * w;
		}

		void Normalize(float tolerance = agm::EPSILON)
		{
			*this = GetNormalized(tolerance);
		}

		Quaternion GetNormalized(float tolerance = agm::EPSILON) const
		{
			float lengthSq = LengthSquared();
			if (lengthSq > tolerance)
			{
				return *this * agm::InvSqrt(lengthSq);
			}
			else
			{
				return Quaternion::IDENTITY;
			}
		}

		constexpr bool IsNormalized(float threshold = agm::QUATERNION_NORMALIZED_THRESHOLD) const
		{
			return agm::Abs(1.f - LengthSquared()) < threshold;
		}

		Vector3 GetEulerAngles() const
		{
			Vector3 euler;

			float sinPitch = 2.f * (w * x - y * z);
			if (sinPitch >= 1.f)
			{
				euler.x = agm::HALF_PI;
			}
			else if (sinPitch <= -1.f)
			{
				euler.x = -agm::HALF_PI;
			}
			else
			{
				euler.x = std::asin(sinPitch);
			}

			if (agm::Abs(sinPitch) < 0.99999f)
			{
				euler.y = std::atan2(2.f * (w * y + x * z), 1.f - 2.f * (x * x + y * y));
				euler.z = std::atan2(2.f * (w * z + x * y), 1.f - 2.f * (x * x + z * z));
			}
			else
			{
				euler.y = std::atan2(2.f * y * w - 2.f * x * z, 1.f - 2.f * y * y - 2.f * z * z);
				euler.z = 0.f;
			}

			euler *= agm::RAD2DEG;
			euler.x = agm::WrapAngle(euler.x);
			euler.y = agm::WrapAngle(euler.y);
			euler.z = agm::WrapAngle(euler.z);

			return euler;
		}

		Vector3 GetRotationAxis() const
		{
			float sinSqHalfAngle = 1.f - w * w;
			if (sinSqHalfAngle <= 0.f)
			{
				return Vector3::FORWARD;
			}

			float invSinHalfAngle = agm::InvSqrt(sinSqHalfAngle);
			return Vector3(x * invSinHalfAngle, y * invSinHalfAngle, z * invSinHalfAngle);
		}

		void ToAngleAxis(float& outAngle, Vector3& outAxis) const
		{
			float clampedW = agm::Clamp(w, -1.f, 1.f);
			outAngle = 2.f * std::acos(clampedW) * agm::RAD2DEG;
			outAxis = GetRotationAxis();
		}

		constexpr Vector3 RotateVector3(const Vector3& v) const
		{
			Vector3 qv(x, y, z);
			Vector3 t = Vector3::Cross(qv, v) * 2.f;
			return v + (t * w) + Vector3::Cross(qv, t);
		}

		constexpr Vector3 UnrotateVector3(const Vector3& v) const
		{
			Vector3 qv(-x, -y, -z);
			float qw = w;
			Vector3 t = Vector3::Cross(qv, v) * 2.f;
			return v + (t * qw) + Vector3::Cross(qv, t);
		}

		constexpr Vector3 GetAxisX() const
		{
			return RotateVector3(Vector3::RIGHT);
		}

		constexpr Vector3 GetAxisY() const
		{
			return RotateVector3(Vector3::UP);
		}

		constexpr Vector3 GetAxisZ() const
		{
			return RotateVector3(Vector3::FORWARD);
		}

		void SetEulerAngles(const Vector3& euler)
		{
			*this = Euler(euler.x, euler.y, euler.z);
		}

		void SetFromToRotation(const Vector3& fromDirection, const Vector3& toDirection)
		{
			*this = FromToRotation(fromDirection, toDirection);
		}

		void SetLookRotation(const Vector3& view, const Vector3& up = Vector3::UP)
		{
			*this = LookRotation(view, up);
		}

		constexpr void Set(float x, float y, float z, float w)
		{
			this->x = x;
			this->y = y;
			this->z = z;
			this->w = w;
		}

		constexpr bool IsSameRotation(const Quaternion& other, float tolerance = agm::EPSILON) const
		{
			return agm::Abs(agm::Abs(Dot(*this, other)) - 1.f) < tolerance;
		}

		constexpr bool Equals(const Quaternion& other, float tolerance = agm::EPSILON) const
		{
			return
				agm::IsNearlyEqual(x, other.x, tolerance) &&
				agm::IsNearlyEqual(y, other.y, tolerance) &&
				agm::IsNearlyEqual(z, other.z, tolerance) &&
				agm::IsNearlyEqual(w, other.w, tolerance);
		}

		std::string ToString() const
		{
			return std::format("({:.5f}, {:.5f}, {:.5f}, {:.5f})", x, y, z, w);
		}

	public:

		static float Angle(const Quaternion& a, const Quaternion& b)
		{
			float cosHalfAngle = agm::Abs(Dot(a.GetNormalized(), b.GetNormalized()));
			float clampedCosHalfAngle = agm::Clamp(cosHalfAngle, -1.f, 1.f);
			return std::acos(clampedCosHalfAngle) * 2.f * agm::RAD2DEG;
		}

		static Quaternion AngleAxis(float angle, const Vector3& axis)
		{
			if (axis.IsNearlyZero())
			{
				return IDENTITY;
			}

			Vector3 normAxis = axis.GetNormalized();
			float halfAngleRad = angle * agm::DEG2RAD * 0.5f;
			float s = std::sin(halfAngleRad);
			float c = std::cos(halfAngleRad);

			return Quaternion(normAxis.x * s, normAxis.y * s, normAxis.z * s, c);
		}

		static constexpr Quaternion Conjugate(const Quaternion& q)
		{
			return Quaternion(-q.x, -q.y, -q.z, q.w);
		}

		static constexpr float Dot(const Quaternion& a, const Quaternion& b)
		{
			return a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
		}

		static Quaternion Euler(float x, float y, float z)
		{
			Quaternion qX = AngleAxis(x, Vector3::RIGHT);
			Quaternion qY = AngleAxis(y, Vector3::UP);
			Quaternion qZ = AngleAxis(z, Vector3::FORWARD);
			return qY * qX * qZ;
		}

		static Quaternion Euler(const Vector3& euler)
		{
			return Euler(euler.x, euler.y, euler.z);
		}

		static Quaternion FromToRotation(const Vector3& fromDirection, const Vector3& toDirection)
		{
			Vector3 fn = fromDirection.GetNormalized();
			Vector3 tn = toDirection.GetNormalized();
			float dot = Vector3::Dot(fn, tn);

			if (dot >= 1.f - agm::EPSILON)
			{
				return Quaternion::IDENTITY;
			}
			else if (dot <= -1.f + agm::EPSILON)
			{
				Vector3 axis = Vector3::Cross(Vector3::RIGHT, fn);
				if (axis.IsNearlyZero(agm::EPSILON_SQUARED))
				{
					axis = Vector3::Cross(Vector3::UP, fn);
				}

				return AngleAxis(180.f, axis);
			}

			float sSqrt = agm::Sqrt((1.f + dot) * 2.f);
			float invS = 1.f / sSqrt;
			Vector3 c = Vector3::Cross(fn, tn);

			return Quaternion(c.x * invS, c.y * invS, c.z * invS, sSqrt * 0.5f);
		}

		static Quaternion Inverse(const Quaternion& q)
		{
			float lenSq = q.LengthSquared();
			if (agm::IsNearlyZero(lenSq, agm::EPSILON_SQUARED))
			{
				return Quaternion::IDENTITY;
			}

			if (agm::Abs(lenSq - 1.f) < agm::QUATERNION_NORMALIZED_THRESHOLD)
			{
				return Conjugate(q);
			}

			return Conjugate(q) * (1.f / lenSq);
		}

		static Quaternion Lerp(const Quaternion& a, const Quaternion& b, float t)
		{
			float clampedT = agm::Clamp01(t);
			Quaternion result;

			if (Dot(a, b) >= 0.f)
			{
				result = a * (1.f - clampedT) + b * clampedT;
			}
			else
			{
				result = a * (1.f - clampedT) - b * clampedT;
			}

			return result.GetNormalized();
		}

		static Quaternion LookRotation(const Vector3& forward, const Vector3& upwards = Vector3::UP)
		{
			if (forward.IsNearlyZero(agm::EPSILON_SQUARED))
			{
				return IDENTITY;
			}

			Vector3 f = forward.GetNormalized();
			Vector3 r = Vector3::Cross(upwards, f).GetNormalized(agm::EPSILON_SQUARED);

			if (r.IsNearlyZero(agm::EPSILON_SQUARED))
			{
				Vector3 fallbackUp = (agm::Abs(f.y) < 0.99999f) ? Vector3::UP : Vector3::FORWARD;
				r = Vector3::Cross(fallbackUp, f).GetNormalized(agm::EPSILON_SQUARED);
				if (r.IsNearlyZero())
				{
					fallbackUp = Vector3::RIGHT;
					r = Vector3::Cross(fallbackUp, f).GetNormalized();
					if (r.IsNearlyZero(agm::EPSILON_SQUARED))
					{
						return Quaternion::IDENTITY;
					}
				}
			}

			Vector3 up = Vector3::Cross(f, r);

			float m00 = r.x;
			float m01 = up.x;
			float m02 = f.x;
			float m10 = r.y;
			float m11 = up.y;
			float m12 = f.y;
			float m20 = r.z;
			float m21 = up.z;
			float m22 = f.z;

			float trace = m00 + m11 + m22;
			Quaternion q;

			if (trace > 0.f)
			{
				float s = agm::Sqrt(trace + 1.f) * 2.f;
				q.w = 0.25f * s;
				q.x = (m21 - m12) / s;
				q.y = (m02 - m20) / s;
				q.z = (m10 - m01) / s;
			}
			else if ((m00 > m11) && (m00 > m22))
			{
				float s = agm::Sqrt(1.f + m00 - m11 - m22) * 2.f;
				q.w = (m21 - m12) / s;
				q.x = 0.25f * s;
				q.y = (m01 + m10) / s;
				q.z = (m02 + m20) / s;
			}
			else if (m11 > m22)
			{
				float s = agm::Sqrt(1.f + m11 - m00 - m22) * 2.f;
				q.w = (m02 - m20) / s;
				q.x = (m01 + m10) / s;
				q.y = 0.25f * s;
				q.z = (m12 + m21) / s;
			}
			else
			{
				float s = agm::Sqrt(1.f + m22 - m00 - m11) * 2.f;
				q.w = (m10 - m01) / s;
				q.x = (m02 + m20) / s;
				q.y = (m12 + m21) / s;
				q.z = 0.25f * s;
			}

			return q.GetNormalized();
		}

		static Quaternion RotateTowards(const Quaternion& from, const Quaternion& to, float maxAngleDelta)
		{
			if (maxAngleDelta <= 0.f)
			{
				return from;
			}

			float angle = Angle(from, to);
			if (angle <= maxAngleDelta)
			{
				return to;
			}

			float t = maxAngleDelta / angle;
			return Slerp(from, to, t);
		}

		static Quaternion Slerp(const Quaternion& a, const Quaternion& b, float t)
		{
			float clampedT = agm::Clamp01(t);

			Quaternion an = a.GetNormalized();
			Quaternion bn = b.GetNormalized();

			float cosHalfTheta = Dot(an, bn);
			Quaternion shortestB = bn;

			if (cosHalfTheta < 0.f)
			{
				shortestB = -bn;
				cosHalfTheta = -cosHalfTheta;
			}

			if (cosHalfTheta > 1.f - agm::EPSILON)
			{
				return Lerp(an, shortestB, clampedT);
			}

			float halfTheta = std::acos(cosHalfTheta);
			float sinHalfTheta = std::sin(halfTheta);

			if (agm::IsNearlyZero(sinHalfTheta))
			{
				return Lerp(an, shortestB, clampedT);
			}

			float s0 = std::sin((1.f - clampedT) * halfTheta) / sinHalfTheta;
			float s1 = std::sin(clampedT * halfTheta) / sinHalfTheta;

			Quaternion result = (an * s0) + (shortestB * s1);
			return result.GetNormalized();
		}
	};

	inline const Quaternion Quaternion::IDENTITY = Quaternion(0.f, 0.f, 0.f, 1.f);
}

