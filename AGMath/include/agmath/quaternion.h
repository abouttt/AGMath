#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <format>
#include <string>

#include "utilities.h"
#include "vector3.h"

namespace agm
{
	struct Quaternion
	{
		static const Quaternion IDENTITY;

		union
		{
			struct
			{
				float x, y, z, w;
			};

			std::array<float, 4> data;
		};

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

		inline constexpr float operator[](size_t index) const
		{
			return data[index];
		}

		inline constexpr float& operator[](size_t index)
		{
			return data[index];
		}

		inline constexpr Quaternion operator-() const
		{
			return Quaternion(-x, -y, -z, -w);
		}

		inline constexpr Quaternion operator*(float scalar) const
		{
			return Quaternion(x * scalar, y * scalar, z * scalar, w * scalar);
		}

		inline constexpr Quaternion operator/(float scalar) const
		{
			return Quaternion(x / scalar, y / scalar, z / scalar, w / scalar);
		}

		inline constexpr Quaternion operator+(const Quaternion& other) const
		{
			return Quaternion(x + other.x, y + other.y, z + other.z, w + other.w);
		}

		inline constexpr Quaternion operator-(const Quaternion& other) const
		{
			return Quaternion(x - other.x, y - other.y, z - other.z, w - other.w);
		}

		inline constexpr Quaternion operator*(const Quaternion& other) const
		{
			return Quaternion(
				w * other.x + x * other.w + y * other.z - z * other.y,
				w * other.y + y * other.w + z * other.x - x * other.z,
				w * other.z + z * other.w + x * other.y - y * other.x,
				w * other.w - x * other.x - y * other.y - z * other.z
			);
		}

		inline constexpr Vector3 operator*(const Vector3& point) const
		{
			if (!IsNormalized())
			{
				return Normalized() * point;
			}

			Vector3 qv(x, y, z);
			Vector3 uv = Vector3::Cross(qv, point);
			Vector3 uuv = Vector3::Cross(qv, uv);
			return point + (uv * w + uuv) * 2.f;
		}

		inline constexpr Quaternion& operator*=(float scalar)
		{
			x *= scalar;
			y *= scalar;
			z *= scalar;
			w *= scalar;
			return *this;
		}

		inline constexpr Quaternion& operator/=(float scalar)
		{
			x /= scalar;
			y /= scalar;
			z /= scalar;
			w /= scalar;
			return *this;
		}

		inline constexpr Quaternion& operator+=(const Quaternion& other)
		{
			x += other.x;
			y += other.y;
			z += other.z;
			w += other.w;
			return *this;
		}

		inline constexpr Quaternion& operator-=(const Quaternion& other)
		{
			x -= other.x;
			y -= other.y;
			z -= other.z;
			w -= other.w;
			return *this;
		}

		inline constexpr Quaternion& operator*=(const Quaternion& other)
		{
			*this = *this * other;
			return *this;
		}

		inline constexpr bool operator==(const Quaternion& other) const
		{
			return x == other.x && y == other.y && z == other.z && w == other.w;
		}

		inline constexpr bool operator!=(const Quaternion& other) const
		{
			return !(*this == other);
		}

		static inline float Angle(const Quaternion& a, const Quaternion& b)
		{
			float dotAB = Min(Abs(Dot(a, b)), 1.f);
			return dotAB > 0.999999f ? 0.f : std::acos(dotAB) * 2.f * RAD2DEG;
		}

		static inline Quaternion AngleAxis(float angle, const Vector3& axis)
		{
			float lengthSq = axis.LengthSquared();
			if (lengthSq < EPSILON)
			{
				return Quaternion::IDENTITY;
			}

			Vector3 normAxis = axis / std::sqrt(lengthSq);
			float halfAngle = angle * DEG2RAD * 0.5f;
			float sinHalf = std::sin(halfAngle);
			return Quaternion(
				normAxis.x * sinHalf,
				normAxis.y * sinHalf,
				normAxis.z * sinHalf,
				std::cos(halfAngle)
			).Normalized();
		}

		static inline constexpr Quaternion Conjugate(const Quaternion& q)
		{
			return Quaternion(-q.x, -q.y, -q.z, q.w);
		}

		static inline constexpr float Dot(const Quaternion& a, const Quaternion& b)
		{
			return a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
		}

		static inline Quaternion Euler(float x, float y, float z)
		{
			x *= DEG2RAD * 0.5f;
			y *= DEG2RAD * 0.5f;
			z *= DEG2RAD * 0.5f;

			float cx = std::cos(x);
			float sx = std::sin(x);
			float cy = std::cos(y);
			float sy = std::sin(y);
			float cz = std::cos(z);
			float sz = std::sin(z);

			return Quaternion(
				cy * sx * cz + sy * cx * sz,
				sy * cx * cz - cy * sx * sz,
				cy * cx * sz - sy * sx * cz,
				cy * cx * cz + sy * sx * sz
			);
		}

		static inline Quaternion Euler(const Vector3& euler)
		{
			return Quaternion::Euler(euler.x, euler.y, euler.z);
		}

		static inline Quaternion FromToRotation(const Vector3& fromDirection, const Vector3& toDirection)
		{
			if (fromDirection.LengthSquared() < EPSILON || toDirection.LengthSquared() < EPSILON)
			{
				return Quaternion::IDENTITY;
			}

			Vector3 v0 = fromDirection.Normalized();
			Vector3 v1 = toDirection.Normalized();

			float dot = Vector3::Dot(v0, v1);
			Vector3 cross = Vector3::Cross(v0, v1);

			Quaternion rotation;
			rotation.w = 1.f + dot;
			rotation.x = cross.x;
			rotation.y = cross.y;
			rotation.z = cross.z;

			return rotation.Normalized();
		}

		static constexpr Quaternion Inverse(const Quaternion& q)
		{
			return q.IsNormalized() ? Conjugate(q) : Conjugate(q) / Dot(q, q);
		}

		static inline Quaternion Lerp(const Quaternion& a, const Quaternion& b, float t)
		{
			t = Clamp01(t);
			return LerpUnclamped(a, b, t);
		}

		static inline Quaternion LerpUnclamped(const Quaternion& a, const Quaternion& b, float t)
		{
			Quaternion result = (Dot(a, b) >= 0.f) ? a * (1.f - t) + b * t : a * (1.f - t) - b * t;
			return result.Normalized();
		}

		static inline Quaternion LookRotation(const Vector3& forward, const Vector3& upwards = Vector3::UP)
		{
			Vector3 f = forward.Normalized();
			Vector3 u = upwards.Normalized();

			if (Vector3::Cross(f, u).LengthSquared() < EPSILON)
			{
				u = (Abs(f.y) > 0.999f) ? Vector3::RIGHT : Vector3::UP;
			}

			Vector3 r = Vector3::Cross(u, f).Normalized();
			u = Vector3::Cross(f, r).Normalized();

			float m00 = r.x;
			float m01 = r.y;
			float m02 = r.z;
			float m10 = u.x;
			float m11 = u.y;
			float m12 = u.z;
			float m20 = f.x;
			float m21 = f.y;
			float m22 = f.z;

			float trace = m00 + m11 + m22;
			Quaternion rotation;

			if (trace > 0)
			{
				float s = std::sqrt(trace + 1.f) * 2.f;
				rotation.w = 0.25f * s;
				rotation.x = (m21 - m12) / s;
				rotation.y = (m02 - m20) / s;
				rotation.z = (m10 - m01) / s;
			}
			else if ((m00 > m11) && (m00 > m22))
			{
				float s = std::sqrt(1.f + m00 - m11 - m22) * 2.f;
				rotation.w = (m21 - m12) / s;
				rotation.x = 0.25f * s;
				rotation.y = (m01 + m10) / s;
				rotation.z = (m02 + m20) / s;
			}
			else if (m11 > m22)
			{
				float s = std::sqrt(1.f + m11 - m00 - m22) * 2.f;
				rotation.w = (m02 - m20) / s;
				rotation.x = (m01 + m10) / s;
				rotation.y = 0.25f * s;
				rotation.z = (m12 + m21) / s;
			}
			else
			{
				float s = std::sqrt(1.f + m22 - m00 - m11) * 2.f;
				rotation.w = (m10 - m01) / s;
				rotation.x = (m02 + m20) / s;
				rotation.y = (m12 + m21) / s;
				rotation.z = 0.25f * s;
			}

			return rotation;
		}

		static inline Quaternion RotateTowards(const Quaternion& from, const Quaternion& to, float maxDegreesDelta)
		{
			float angle = Angle(from, to);
			if (angle < EPSILON)
			{
				return to;
			}

			return SlerpUnclamped(from, to, Min(1.f, maxDegreesDelta / angle));
		}

		static inline Quaternion Slerp(const Quaternion& a, const Quaternion& b, float t)
		{
			t = Clamp01(t);
			return SlerpUnclamped(a, b, t);
		}

		static inline Quaternion SlerpUnclamped(const Quaternion& a, const Quaternion& b, float t)
		{
			float dot = Dot(a, b);
			if (dot < 0.f)
			{
				return SlerpUnclamped(a, -b, t);
			}
			if (dot > 0.9995f)
			{
				Quaternion result = a + (b - a) * t;
				return result.Normalized();
			}

			float theta0 = std::acos(dot);
			float theta = theta0 * t;
			Quaternion bTemp = (b - a * dot).Normalized();

			return a * std::cos(theta) + bTemp * std::sin(theta);
		}

		inline Vector3 EulerAngles() const
		{
			float sinX = 2.f * (w * x - y * z);
			float xRad = std::asin(Clamp(sinX, -1.f, 1.f));

			float sinY_cosX = 2.f * (w * y + z * x);
			float cosY_cosX = 1.f - 2.f * (x * x + y * y);
			float yRad = std::atan2(sinY_cosX, cosY_cosX);

			float sinZ_cosX = 2.f * (w * z + x * y);
			float cosZ_cosX = 1.f - 2.f * (z * z + x * x);
			float zRad = std::atan2(sinZ_cosX, cosZ_cosX);

			Vector3 euler(xRad, yRad, zRad);
			euler *= RAD2DEG;

			if (euler.x < 0.f)
			{
				euler.x += 360.f;
			}
			if (euler.y < 0.f)
			{
				euler.y += 360.f;
			}
			if (euler.z < 0.f)
			{
				euler.z += 360.f;
			}

			return euler;
		}

		inline void SetEulerAngles(const Vector3& euler)
		{
			*this = Euler(euler.x, euler.y, euler.z);
		}

		inline float Length() const
		{
			return std::sqrt(x * x + y * y + z * z + w * w);
		}

		inline constexpr float LengthSquared() const
		{
			return x * x + y * y + z * z + w * w;
		}

		inline constexpr bool IsNormalized() const
		{
			return Abs(1.f - LengthSquared()) < 0.01f;
		}

		inline void Normalize(float tolerance = EPSILON)
		{
			*this = Normalized(tolerance);
		}

		inline Quaternion Normalized(float tolerance = EPSILON) const
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

		inline constexpr bool Equals(const Quaternion& other, float tolerance = LOOSE_EPSILON) const
		{
			return Abs(x - other.x) <= tolerance && Abs(y - other.y) <= tolerance && Abs(z - other.z) <= tolerance && Abs(w - other.w) <= tolerance;
		}

		inline constexpr void Set(float x, float y, float z, float w)
		{
			this->x = x;
			this->y = y;
			this->z = z;
			this->w = w;
		}

		inline void SetFromToRotation(const Vector3& fromDirection, const Vector3& toDirection)
		{
			*this = FromToRotation(fromDirection, toDirection);
		}

		inline void SetLookRotation(const Vector3& view, const Vector3& up = Vector3::UP)
		{
			*this = LookRotation(view, up);
		}

		inline void ToAngleAxis(float& outAngle, Vector3& outAxis) const
		{
			float wClamped = Clamp(w, -1.f, 1.f);
			outAngle = 2.f * std::acos(wClamped) * RAD2DEG;

			float s = std::sqrt(Max(1.f - wClamped * wClamped, 0.f));
			outAxis = (s >= 0.0001f) ? Vector3(x / s, y / s, z / s) : Vector3::UP;
		}

		inline std::string ToString() const
		{
			return std::format("({:.5f}, {:.5f}, {:.5f}, {:.5f})", x, y, z, w);
		}
	};

	inline const Quaternion Quaternion::IDENTITY = Quaternion(0.f, 0.f, 0.f, 1.f);
}

