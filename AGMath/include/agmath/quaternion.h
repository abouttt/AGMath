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
			, w(0.f)
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

		constexpr float operator[](int32_t index) const
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

		constexpr float& operator[](int32_t index)
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
			*this = (*this) * other;
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

		void Normalize(float tolerance = EPSILON)
		{
			*this = GetNormalized(tolerance);
		}

		Quaternion GetNormalized(float tolerance = EPSILON) const
		{
			float lengthSq = LengthSquared();
			if (lengthSq > tolerance)
			{
				float invLength = 1.f / std::sqrt(lengthSq);
				return *this * invLength;
			}
			else
			{
				return Quaternion::IDENTITY;
			}
		}

		constexpr bool IsNormalized() const
		{
			return Abs(1.f - LengthSquared()) < THRESH_QUAT_NORMALIZED;
		}

		Vector3 GetEulerAngles() const
		{
			Vector3 euler;

			float sinPitch = 2.f * (w * x - y * z);
			sinPitch = Clamp(sinPitch, -1.f, 1.f);
			euler.x = std::asin(sinPitch);

			if (Abs(sinPitch) < 0.99999f)
			{
				euler.y = std::atan2(2.f * (w * y + x * z), 1.f - 2.f * (x * x + y * y));
				euler.z = std::atan2(2.f * (w * z + x * y), 1.f - 2.f * (x * x + z * z));
			}
			else
			{
				euler.y = std::atan2(2.f * y * w - 2.f * x * z, 1.f - 2.f * y * y - 2.f * z * z);
				euler.z = 0.f;
				euler.y = std::atan2(2.f * (x * z + w * y), 1.f - 2.f * (y * y + x * x));
				euler.z = 0.f;
			}

			euler *= RAD2DEG;

			for (int i = 0; i < 3; i++)
			{
				float& angle = euler[i];
				angle = std::fmod(angle, 360.f);
				if (angle < 0.f)
				{
					angle += 360.f;
				}
			}

			return euler;
		}

		Vector3 GetRotationAxis() const
		{
			float s = std::sqrt(Max(1.f - (w * w), 0.f));

			if (s >= 0.0001f)
			{
				return Vector3(x / s, y / s, z / s);
			}

			return Vector3::RIGHT;
		}

		void ToAngleAxis(float& outAngle, Vector3& outAxis) const
		{
			outAngle = 2.f * std::acos(w) * RAD2DEG;
			outAxis = GetRotationAxis();
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

		constexpr Vector3 RotateVector3(const Vector3& v) const
		{
			Vector3 qv(x, y, z);
			Vector3 t = Vector3::Cross(qv, v) * 2.f;
			return v + (t * w) + Vector3::Cross(qv, t);
		}

		constexpr Vector3 UnrotateVector3(const Vector3& v) const
		{
			Vector3 qv = -v;
			Vector3 t = Vector3::Cross(qv, v) * 2.f;
			return v + (t * w) + Vector3::Cross(qv, t);
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

		constexpr bool IsIdentity(float tolerance = EPSILON) const
		{
			return Equals(IDENTITY, tolerance);
		}

		constexpr void Set(float x, float y, float z, float w)
		{
			this->x = x;
			this->y = y;
			this->z = z;
			this->w = w;
		}

		constexpr bool Equals(const Quaternion& other, float tolerance = LOOSE_EPSILON) const
		{
			return Abs(x - other.x) <= tolerance && Abs(y - other.y) <= tolerance && Abs(z - other.z) <= tolerance && Abs(w - other.w) <= tolerance;
		}

		std::string ToString() const
		{
			return std::format("({:.5f}, {:.5f}, {:.5f}, {:.5f})", x, y, z, w);
		}

	public:

		static float Angle(const Quaternion& a, const Quaternion& b)
		{
			Quaternion aNormalized = a.GetNormalized();
			Quaternion bNormalized = b.GetNormalized();

			float dotAB = Dot(aNormalized, bNormalized);
			float clampedDot = Clamp(dotAB, -1.f, 1.f);

			return std::acos(Abs(clampedDot)) * 2.f * RAD2DEG;
		}

		static Quaternion AngleAxis(float angle, const Vector3& axis)
		{
			if (axis.LengthSquared() < EPSILON)
			{
				return Quaternion::IDENTITY;
			}

			Vector3 normalizedAxis = axis.GetNormalized();
			float halfAngleRad = angle * DEG2RAD * 0.5f;
			float s = std::sin(halfAngleRad);
			float c = std::cos(halfAngleRad);

			return Quaternion(
				normalizedAxis.x * s,
				normalizedAxis.y * s,
				normalizedAxis.z * s,
				c
			);
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
			Quaternion qRoll = AngleAxis(z, Vector3::FORWARD);
			Quaternion qPitch = AngleAxis(x, Vector3::RIGHT);
			Quaternion qYaw = AngleAxis(y, Vector3::UP);

			return qYaw * qPitch * qRoll;
		}

		static Quaternion Euler(const Vector3& euler)
		{
			return Quaternion::Euler(euler.x, euler.y, euler.z);
		}

		static Quaternion FromToRotation(const Vector3& fromDirection, const Vector3& toDirection)
		{
			Vector3 v0 = fromDirection.GetNormalized();
			Vector3 v1 = toDirection.GetNormalized();
			float dot = Vector3::Dot(v0, v1);

			if (dot >= 1.f - EPSILON)
			{
				return Quaternion::IDENTITY;
			}
			else if (dot <= -1.f + EPSILON)
			{
				Vector3 orthoAxis = Vector3::Cross(Vector3::RIGHT, v0);
				if (orthoAxis.LengthSquared() < EPSILON)
				{
					orthoAxis = Vector3::Cross(Vector3::UP, v0);
				}
				orthoAxis.Normalize();
				return AngleAxis(180.f, orthoAxis);
			}
			else
			{
				float s = std::sqrt((1.f + dot) * 2.f);
				float invS = 1.f / s;
				Vector3 c = Vector3::Cross(v0, v1);
				return Quaternion(
					c.x * invS,
					c.y * invS,
					c.z * invS,
					s * 0.5f
				).GetNormalized();
			}
		}

		static constexpr Quaternion Inverse(const Quaternion& q)
		{
			float lengthSq = q.LengthSquared();

			if (Abs(lengthSq - 1.f) < 0.01f)
			{
				return Conjugate(q);
			}
			else if (lengthSq < EPSILON)
			{
				return Quaternion::IDENTITY;
			}

			return Conjugate(q) / lengthSq;
		}

		static Quaternion Lerp(const Quaternion& a, const Quaternion& b, float t)
		{
			t = Clamp01(t);
			return LerpUnclamped(a, b, t);
		}

		static Quaternion LerpUnclamped(const Quaternion& a, const Quaternion& b, float t)
		{
			Quaternion result;

			if (Dot(a, b) >= 0.f)
			{
				result = a * (1.f - t) + b * t;
			}
			else
			{
				result = a * (1.f - t) - b * t;
			}

			return result.GetNormalized();
		}

		static Quaternion LookRotation(const Vector3& forward, const Vector3& upwards = Vector3::UP)
		{
			if (forward.LengthSquared() < EPSILON)
			{
				return Quaternion::IDENTITY;
			}

			Vector3 f = forward.GetNormalized();
			Vector3 r = Vector3::Cross(upwards, f).GetNormalized();

			if (r.LengthSquared() < EPSILON)
			{
				if (Abs(f.y) > 1.f - EPSILON)
				{
					r = Vector3::Cross(Vector3::FORWARD, f).GetNormalized();
				}
				else
				{
					r = Vector3::Cross(Vector3::RIGHT, f).GetNormalized();
				}
			}

			Vector3 u = Vector3::Cross(f, r);

			float m00 = r.x, m01 = u.x, m02 = f.x;
			float m10 = r.y, m11 = u.y, m12 = f.y;
			float m20 = r.z, m21 = u.z, m22 = f.z;

			float trace = m00 + m11 + m22;
			Quaternion q;

			if (trace > 0.f)
			{
				float s = 0.5f / std::sqrt(trace + 1.f);
				q.w = 0.25f / s;
				q.x = (m21 - m12) * s;
				q.y = (m02 - m20) * s;
				q.z = (m10 - m01) * s;
			}
			else
			{
				if (m00 > m11 && m00 > m22)
				{
					float s = 2.f * std::sqrt(1.f + m00 - m11 - m22);
					q.w = (m21 - m12) / s;
					q.x = 0.25f * s;
					q.y = (m01 + m10) / s;
					q.z = (m02 + m20) / s;
				}
				else if (m11 > m22)
				{
					float s = 2.f * std::sqrt(1.f + m11 - m00 - m22);
					q.w = (m02 - m20) / s;
					q.x = (m01 + m10) / s;
					q.y = 0.25f * s;
					q.z = (m12 + m21) / s;
				}
				else
				{
					float s = 2.f * std::sqrt(1.f + m22 - m00 - m11);
					q.w = (m10 - m01) / s;
					q.x = (m02 + m20) / s;
					q.y = (m12 + m21) / s;
					q.z = 0.25f * s;
				}
			}

			return q.GetNormalized();
		}

		static Quaternion RotateTowards(const Quaternion& from, const Quaternion& to, float maxDegreesDelta)
		{
			float angle = Angle(from, to);

			if (angle < EPSILON || maxDegreesDelta <= 0.f)
			{
				return from;
			}
			if (maxDegreesDelta >= angle)
			{
				return to.GetNormalized();
			}

			return SlerpUnclamped(from, to, maxDegreesDelta / angle);
		}

		static Quaternion Slerp(const Quaternion& a, const Quaternion& b, float t)
		{
			t = Clamp01(t);
			return SlerpUnclamped(a, b, t);
		}

		static Quaternion SlerpUnclamped(const Quaternion& a, const Quaternion& b, float t)
		{
			Quaternion an = a.GetNormalized();
			Quaternion bn = b.GetNormalized();

			float cosHalfTheta = Dot(an, bn);

			if (cosHalfTheta < 0.f)
			{
				bn = -bn;
				cosHalfTheta = -cosHalfTheta;
			}

			if (cosHalfTheta > 1.f - EPSILON)
			{
				Quaternion result = an * (1.f - t) + bn * t;
				return result.GetNormalized();
			}

			float halfTheta = std::acos(cosHalfTheta);
			float sinHalfTheta = std::sin(halfTheta);

			if (Abs(sinHalfTheta) < EPSILON)
			{
				return (an * (1.f - t) + bn * t).GetNormalized();
			}

			float s0 = std::sin((1.f - t) * halfTheta) / sinHalfTheta;
			float s1 = std::sin(t * halfTheta) / sinHalfTheta;

			return (an * s0) + (bn * s1);
		}
	};

	inline const Quaternion Quaternion::IDENTITY = Quaternion(0.f, 0.f, 0.f, 1.f);
}

