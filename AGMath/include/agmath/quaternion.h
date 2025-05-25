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
			*this = *this * other;
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

		friend constexpr Quaternion operator*(float scalar, const Quaternion& q)
		{
			return q * scalar;
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

		void Normalize(float tolerance = agm::EPSILON_SQ_LENGTH)
		{
			*this = GetNormalized(tolerance);
		}

		Quaternion GetNormalized(float tolerance = agm::EPSILON_SQ_LENGTH) const
		{
			float lenSq = LengthSquared();
			if (lenSq > tolerance)
			{
				return *this * agm::InvSqrt(lenSq);
			}
			else
			{
				return IDENTITY;
			}
		}

		bool IsNormalized(float threshold = agm::QUATERNION_NORMALIZED_THRESHOLD) const
		{
			return agm::Abs(1.f - LengthSquared()) < threshold;
		}

		Vector3 ToEulerAngles() const
		{
			Vector3 euler;
			float sinPitch = 2.f * (w * x - y * z);

			if (sinPitch >= 1.f - agm::EPSILON_DOT_ONE)
			{
				euler.x = agm::HALF_PI;
			}
			else if (sinPitch <= -1.f + agm::EPSILON_DOT_ONE)
			{
				euler.x = -agm::HALF_PI;
			}
			else
			{
				euler.x = agm::Asin(sinPitch);
			}

			if (agm::Abs(sinPitch) < 1.f - agm::EPSILON_DOT_ONE)
			{
				euler.y = agm::Atan2(2.f * (w * y + x * z), 1.f - 2.f * (x * x + y * y));
				euler.z = agm::Atan2(2.f * (w * z + x * y), 1.f - 2.f * (x * x + z * z));
			}
			else
			{
				euler.y = agm::Atan2(2.f * (y * w + x * z), 1.f - 2.f * (y * y + z * z));
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
			if (sinSqHalfAngle <= agm::EPSILON_SQ_LENGTH)
			{
				return Vector3::FORWARD;
			}

			float invSinHalfAngle = agm::InvSqrt(sinSqHalfAngle);
			return Vector3(x * invSinHalfAngle, y * invSinHalfAngle, z * invSinHalfAngle);
		}

		void ToAngleAxis(float& outAngle, Vector3& outAxis) const
		{
			float clampedW = agm::Clamp(w, -1.f, 1.f);
			outAngle = 2.f * agm::Acos(clampedW) * agm::RAD2DEG;
			outAxis = GetRotationAxis();
		}

		constexpr Vector3 RotateVector3(const Vector3& v) const
		{
			Vector3 quatVec(x, y, z);
			Vector3 temp = Vector3::Cross(quatVec, v) * 2.f;
			return v + (temp * w) + Vector3::Cross(quatVec, temp);
		}

		constexpr Vector3 UnrotateVector3(const Vector3& v) const
		{
			Vector3 conjVec(-x, -y, -z);
			Vector3 temp = Vector3::Cross(conjVec, v) * 2.f;
			return v + (temp * w) + Vector3::Cross(conjVec, temp);
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

		void FromEulerAngles(const Vector3& euler)
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

		constexpr bool IsSameRotation(const Quaternion& other, float tolerance = agm::EPSILON_DOT_ONE) const
		{
			return agm::Abs(agm::Abs(Dot(*this, other)) - 1.f) < tolerance;
		}

		constexpr bool Equals(const Quaternion& other, float tolerance = agm::EPSILON) const
		{
			return agm::IsNearlyEqual(x, other.x, tolerance) &&
				   agm::IsNearlyEqual(y, other.y, tolerance) &&
				   agm::IsNearlyEqual(z, other.z, tolerance) &&
				   agm::IsNearlyEqual(w, other.w, tolerance);
		}

		std::string ToString() const
		{
			return std::format("({:.3f}, {:.3f}, {:.3f}, {:.3f})", x, y, z, w);
		}

	public:

		static float Angle(const Quaternion& a, const Quaternion& b)
		{
			float cosHalfAngle = agm::Abs(Dot(a.GetNormalized(), b.GetNormalized()));
			return agm::Acos(agm::Clamp(cosHalfAngle, -1.f, 1.f)) * 2.f * agm::RAD2DEG;
		}

		static Quaternion AngleAxis(float angleDeg, const Vector3& axis)
		{
			if (axis.IsNearlyZero(agm::EPSILON_SQ_LENGTH))
			{
				return IDENTITY;
			}

			Vector3 normAxis = axis.GetNormalized();
			float halfAngleRad = angleDeg * agm::DEG2RAD * 0.5f;
			float s = agm::Sin(halfAngleRad);
			float c = agm::Cos(halfAngleRad);

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
			Vector3 normFrom = fromDirection.GetNormalized();
			Vector3 normTo = toDirection.GetNormalized();

			float d = Vector3::Dot(normFrom, normTo);
			if (d >= 1.f - agm::EPSILON_DOT_ONE)
			{
				return IDENTITY;
			}
			if (d <= -1.f + agm::EPSILON_DOT_ONE)
			{
				Vector3 axis = Vector3::FindOrthogonal(normFrom);
				if (axis.IsNearlyZero(agm::EPSILON_SQ_LENGTH))
				{
					return AngleAxis(180.f, Vector3::UP);
				}

				return AngleAxis(180.f, axis);
			}

			Vector3 cross = Vector3::Cross(normFrom, normTo);
			return Quaternion(cross.x, cross.y, cross.z, 1.f + d).GetNormalized();
		}

		static Quaternion Inverse(const Quaternion& q)
		{
			float lenSq = q.LengthSquared();
			if (lenSq <= agm::EPSILON_SQ_LENGTH)
			{
				return IDENTITY;
			}
			if (agm::Abs(lenSq - 1.f) < agm::QUATERNION_NORMALIZED_THRESHOLD)
			{
				return Conjugate(q);
			}

			return Conjugate(q) * (1.f / lenSq);
		}

		static Quaternion Lerp(const Quaternion& a, const Quaternion& b, float t)
		{
			t = agm::Clamp01(t);

			Quaternion result;

			if (Dot(a, b) >= 0.f)
			{
				result = a * (1.f - t) + b * t;
			}
			else
			{
				result = a * (1.f - t) - (b * t);
			}

			return result.GetNormalized(agm::EPSILON_SQ_LENGTH);
		}

		static Quaternion LookRotation(const Vector3& forward, const Vector3& up = Vector3::UP)
		{
			if (forward.IsNearlyZero(agm::EPSILON_SQ_LENGTH))
			{
				return IDENTITY;
			}

			Vector3 f = forward.GetNormalized();
			Vector3 r = Vector3::Cross(up, f);
			if (r.IsNearlyZero(agm::EPSILON_SQ_LENGTH))
			{
				if (agm::Abs(f.y) > 1.f - agm::EPSILON_DOT_ONE)
				{
					r = Vector3::Cross(Vector3::FORWARD, f);
				}
				else
				{
					r = Vector3::Cross(Vector3::UP, f);
				}

				if (r.IsNearlyZero(agm::EPSILON_SQ_LENGTH))
				{
					if (agm::Abs(f.x) < 0.9f)
					{
						r = Vector3::RIGHT;
					}
					else
					{
						r = Vector3::UP;
					}
					r = Vector3::Cross(r, f);
					if (r.IsNearlyZero(agm::EPSILON_SQ_LENGTH))
					{
						return IDENTITY;
					}
				}
			}
			r.Normalize();
			Vector3 u = Vector3::Cross(f, r);

			float m00 = r.x;
			float m01 = u.x;
			float m02 = f.x;

			float m10 = r.y;
			float m11 = u.y;
			float m12 = f.y;

			float m20 = r.z;
			float m21 = u.z;
			float m22 = f.z;

			float trace = m00 + m11 + m22;
			Quaternion result;
			if (trace > 0.f)
			{
				float s = agm::Sqrt(trace + 1.f) * 2.f;
				result.w = 0.25f * s;
				result.x = (m21 - m12) / s;
				result.y = (m02 - m20) / s;
				result.z = (m10 - m01) / s;
			}
			else if ((m00 > m11) && (m00 > m22))
			{
				float s = agm::Sqrt(1.f + m00 - m11 - m22) * 2.f;
				result.w = (m21 - m12) / s;
				result.x = 0.25f * s;
				result.y = (m01 + m10) / s;
				result.z = (m02 + m20) / s;
			}
			else if (m11 > m22)
			{
				float s = agm::Sqrt(1.f + m11 - m00 - m22) * 2.f;
				result.w = (m02 - m20) / s;
				result.x = (m01 + m10) / s;
				result.y = 0.25f * s;
				result.z = (m12 + m21) / s;
			}
			else
			{
				float s = agm::Sqrt(1.f + m22 - m00 - m11) * 2.f;
				result.w = (m10 - m01) / s;
				result.x = (m02 + m20) / s;
				result.y = (m12 + m21) / s;
				result.z = 0.25f * s;
			}

			return result.GetNormalized();
		}

		static Quaternion RotateTowards(const Quaternion& from, const Quaternion& to, float maxDegreesDelta)
		{
			if (maxDegreesDelta <= 0.f)
			{
				return from;
			}

			float angle = Angle(from, to);
			if (angle <= maxDegreesDelta || angle < agm::EPSILON)
			{
				return to;
			}

			return Slerp(from, to, maxDegreesDelta / angle);
		}

		static Quaternion Slerp(const Quaternion& a, const Quaternion& b, float t)
		{
			t = agm::Clamp01(t);

			Quaternion normA = a.GetNormalized();
			Quaternion normB = b.GetNormalized();
			Quaternion effectiveB = normB;

			float cosHalfTheta = Dot(normA, normB);
			if (cosHalfTheta < 0.f)
			{
				effectiveB = -normB;
				cosHalfTheta = -cosHalfTheta;
			}
			if (cosHalfTheta > 1.f - agm::EPSILON_DOT_ONE)
			{
				return Lerp(normA, effectiveB, t);
			}

			float halfTheta = agm::Acos(cosHalfTheta);
			float sinHalfTheta = agm::Sin(halfTheta);
			if (agm::IsNearlyZero(sinHalfTheta))
			{
				return Lerp(normA, effectiveB, t);
			}

			float s0 = agm::Sin((1.f - t) * halfTheta) / sinHalfTheta;
			float s1 = agm::Sin(t * halfTheta) / sinHalfTheta;
			Quaternion result = (normA * s0) + (effectiveB * s1);

			return result.GetNormalized();
		}
	};

	inline const Quaternion Quaternion::IDENTITY = Quaternion(0.f, 0.f, 0.f, 1.f);
}

