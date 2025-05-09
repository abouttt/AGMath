#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <format>
#include <string>

#include "utilities.h"
#include "vector3.h"
#include "vector4.h"
#include "quaternion.h"

namespace agm
{
	struct Matrix4x4
	{
		static const Matrix4x4 ZERO;
		static const Matrix4x4 IDENTITY;

		union
		{
			struct
			{
				float m00, m10, m20, m30;
				float m01, m11, m21, m31;
				float m02, m12, m22, m32;
				float m03, m13, m23, m33;
			};

			std::array<float, 16> data;
		};

		constexpr Matrix4x4()
			: m00(0.f), m10(0.f), m20(0.f), m30(0.f)
			, m01(0.f), m11(0.f), m21(0.f), m31(0.f)
			, m02(0.f), m12(0.f), m22(0.f), m32(0.f)
			, m03(0.f), m13(0.f), m23(0.f), m33(0.f)
		{
		}

		constexpr Matrix4x4(const Vector4& column0, const Vector4& column1, const Vector4& column2, const Vector4& column3)
			: m00(column0.x), m10(column0.y), m20(column0.z), m30(column0.w)
			, m01(column1.x), m11(column1.y), m21(column1.z), m31(column1.w)
			, m02(column2.x), m12(column2.y), m22(column2.z), m32(column2.w)
			, m03(column3.x), m13(column3.y), m23(column3.z), m33(column3.w)
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

		inline constexpr float operator()(size_t row, size_t column) const
		{
			return data[row + column * 4];
		}

		inline constexpr float& operator()(size_t row, size_t column)
		{
			return data[row + column * 4];
		}

		inline constexpr Matrix4x4 operator*(float scalar) const
		{
			Matrix4x4 result;

			for (size_t i = 0; i < 16; i++)
			{
				result.data[i] = data[i] * scalar;
			}

			return result;
		}

		inline constexpr Matrix4x4 operator*(const Matrix4x4& other) const
		{
			Matrix4x4 result;

			result.m00 = m00 * other.m00 + m01 * other.m10 + m02 * other.m20 + m03 * other.m30;
			result.m01 = m00 * other.m01 + m01 * other.m11 + m02 * other.m21 + m03 * other.m31;
			result.m02 = m00 * other.m02 + m01 * other.m12 + m02 * other.m22 + m03 * other.m32;
			result.m03 = m00 * other.m03 + m01 * other.m13 + m02 * other.m23 + m03 * other.m33;

			result.m10 = m10 * other.m00 + m11 * other.m10 + m12 * other.m20 + m13 * other.m30;
			result.m11 = m10 * other.m01 + m11 * other.m11 + m12 * other.m21 + m13 * other.m31;
			result.m12 = m10 * other.m02 + m11 * other.m12 + m12 * other.m22 + m13 * other.m32;
			result.m13 = m10 * other.m03 + m11 * other.m13 + m12 * other.m23 + m13 * other.m33;

			result.m20 = m20 * other.m00 + m21 * other.m10 + m22 * other.m20 + m23 * other.m30;
			result.m21 = m20 * other.m01 + m21 * other.m11 + m22 * other.m21 + m23 * other.m31;
			result.m22 = m20 * other.m02 + m21 * other.m12 + m22 * other.m22 + m23 * other.m32;
			result.m23 = m20 * other.m03 + m21 * other.m13 + m22 * other.m23 + m23 * other.m33;

			result.m30 = m30 * other.m00 + m31 * other.m10 + m32 * other.m20 + m33 * other.m30;
			result.m31 = m30 * other.m01 + m31 * other.m11 + m32 * other.m21 + m33 * other.m31;
			result.m32 = m30 * other.m02 + m31 * other.m12 + m32 * other.m22 + m33 * other.m32;
			result.m33 = m30 * other.m03 + m31 * other.m13 + m32 * other.m23 + m33 * other.m33;

			return result;
		}

		inline constexpr Vector4 operator*(const Vector4& v) const
		{
			return Vector4(
				m00 * v.x + m01 * v.y + m02 * v.z + m03 * v.w,
				m10 * v.x + m11 * v.y + m12 * v.z + m13 * v.w,
				m20 * v.x + m21 * v.y + m22 * v.z + m23 * v.w,
				m30 * v.x + m31 * v.y + m32 * v.z + m33 * v.w
			);
		}

		inline constexpr Matrix4x4& operator*=(float scalar)
		{
			for (size_t i = 0; i < 16; i++)
			{
				data[i] *= scalar;
			}

			return *this;
		}

		inline constexpr Matrix4x4& operator*=(const Matrix4x4& other)
		{
			*this = *this * other;
			return *this;
		}

		inline constexpr bool operator==(const Matrix4x4& other)
		{
			for (int i = 0; i < 16; i++)
			{
				if (data[i] != other.data[i])
				{
					return false;
				}
			}

			return true;
		}

		inline constexpr bool operator!=(const Matrix4x4& other)
		{
			return !(*this == other);
		}

		static inline constexpr Matrix4x4 Frustum(float left, float right, float bottom, float top, float zNear, float zFar)
		{
			Matrix4x4 result;

			float invWidth = 1.f / (right - left);
			float invHeight = 1.f / (top - bottom);
			float invDepth = 1.f / (zNear - zFar);

			result.m00 = 2.f * zNear * invWidth;
			result.m01 = 0.f;
			result.m02 = (right + left) * invWidth;
			result.m03 = 0.f;

			result.m10 = 0.f;
			result.m11 = 2.f * zNear * invHeight;
			result.m12 = (top + bottom) * invHeight;
			result.m13 = 0.f;

			result.m20 = 0.f;
			result.m21 = 0.f;
			result.m22 = 2.f * zFar * zNear * invDepth;
			result.m23 = 2.f * zFar * zNear * invDepth;

			result.m30 = 0.f;
			result.m31 = 0.f;
			result.m32 = -1.f;
			result.m33 = 0.f;

			return result;
		}

		static inline constexpr bool Inverse3DAffine(const Matrix4x4& input, Matrix4x4& output)
		{
			float det = input.m00 * (input.m11 * input.m22 - input.m12 * input.m21) -
				input.m01 * (input.m10 * input.m22 - input.m12 * input.m20) +
				input.m02 * (input.m10 * input.m21 - input.m11 * input.m20);

			if (Abs(det) <= EPSILON)
			{
				return false;
			}

			float invDet = 1.f / det;

			output.m00 = (input.m11 * input.m22 - input.m12 * input.m21) * invDet;
			output.m01 = (input.m02 * input.m21 - input.m01 * input.m22) * invDet;
			output.m02 = (input.m01 * input.m12 - input.m02 * input.m11) * invDet;
			output.m03 = -(output.m00 * input.m03 + output.m01 * input.m13 + output.m02 * input.m23);

			output.m10 = (input.m12 * input.m20 - input.m10 * input.m22) * invDet;
			output.m11 = (input.m00 * input.m22 - input.m02 * input.m20) * invDet;
			output.m12 = (input.m02 * input.m10 - input.m00 * input.m12) * invDet;
			output.m13 = -(output.m10 * input.m03 + output.m11 * input.m13 + output.m12 * input.m23);

			output.m20 = (input.m10 * input.m21 - input.m11 * input.m20) * invDet;
			output.m21 = (input.m01 * input.m20 - input.m00 * input.m21) * invDet;
			output.m22 = (input.m00 * input.m11 - input.m01 * input.m10) * invDet;
			output.m23 = -(output.m20 * input.m03 + output.m21 * input.m13 + output.m22 * input.m23);

			output.m30 = 0.f;
			output.m31 = 0.f;
			output.m32 = 0.f;
			output.m33 = 1.f;

			return true;
		}

		static inline Matrix4x4 LookAt(const Vector3& from, const Vector3& to, const Vector3& up)
		{
			Vector3 z = (from - to).Normalized();
			if (z.LengthSquared() < EPSILON)
			{
				z = Vector3::FORWARD;
			}

			Vector3 x = Vector3::Cross(up, z).Normalized();
			if (x.LengthSquared() < EPSILON)
			{
				x = Vector3::Cross(z, Vector3::UP).Normalized();
				if (x.LengthSquared() < EPSILON)
				{
					x = Vector3::RIGHT;
				}
			}

			Vector3 y = Vector3::Cross(z, x).Normalized();

			Matrix4x4 result;

			result.m00 = x.x;
			result.m01 = x.y;
			result.m02 = x.z;
			result.m03 = -Vector3::Dot(x, from);

			result.m10 = y.x;
			result.m11 = y.y;
			result.m12 = y.z;
			result.m13 = -Vector3::Dot(y, from);

			result.m20 = z.x;
			result.m21 = z.y;
			result.m22 = z.z;
			result.m23 = -Vector3::Dot(z, from);

			result.m30 = 0.f;
			result.m31 = 0.f;
			result.m32 = 0.f;
			result.m33 = 1.f;

			return result;
		}

		static inline constexpr Matrix4x4 Ortho(float left, float right, float bottom, float top, float zNear, float zFar)
		{
			Matrix4x4 result;
			float invWidth = 1.f / (right - left);
			float invHeight = 1.f / (top - bottom);
			float invDepth = 1.f / (zFar - zNear);

			result.m00 = 2.f * invWidth;
			result.m01 = 0.f;
			result.m02 = 0.f;
			result.m03 = -(right + left) * invWidth;

			result.m10 = 0.f;
			result.m11 = 2.f * invHeight;
			result.m12 = 0.f;
			result.m13 = -(top + bottom) * invHeight;

			result.m20 = 0.f;
			result.m21 = 0.f;
			result.m22 = -2.f * invDepth;
			result.m23 = -(zFar + zNear) * invDepth;

			result.m30 = 0.f;
			result.m31 = 0.f;
			result.m32 = 0.f;
			result.m33 = 1.f;

			return result;
		}

		static inline Matrix4x4 Perspective(float fov, float aspect, float zNear, float zFar)
		{
			Matrix4x4 result;
			float tanHalfFov = std::tan(fov * 0.5f * DEG2RAD);
			float invDepth = 1.f / (zNear - zFar);

			result.m00 = 1.f / (aspect * tanHalfFov);
			result.m01 = 0.f;
			result.m02 = 0.f;
			result.m03 = 0.f;

			result.m10 = 0.f;
			result.m11 = 1.f / tanHalfFov;
			result.m12 = 0.f;
			result.m13 = 0.f;

			result.m20 = 0.f;
			result.m21 = 0.f;
			result.m22 = (zFar + zNear) * invDepth;
			result.m23 = 2.f * zFar * zNear * invDepth;

			result.m30 = 0.f;
			result.m31 = 0.f;
			result.m32 = -1.f;
			result.m33 = 0.f;

			return result;
		}

		static inline Matrix4x4 Translate(const Vector3& v)
		{
			Matrix4x4 result = Matrix4x4::IDENTITY;

			result.m03 = v.x;
			result.m13 = v.y;
			result.m23 = v.z;

			return result;
		}

		static inline constexpr Matrix4x4 Rotate(const Quaternion& rotation)
		{
			float x = rotation.x * 2.f;
			float y = rotation.y * 2.f;
			float z = rotation.z * 2.f;
			float xx = rotation.x * x;
			float yy = rotation.y * y;
			float zz = rotation.z * z;
			float xy = rotation.x * y;
			float xz = rotation.x * z;
			float yz = rotation.y * z;
			float wx = rotation.w * x;
			float wy = rotation.w * y;
			float wz = rotation.w * z;

			Matrix4x4 result;

			result.m00 = 1.f - (yy + zz);
			result.m01 = xy - wz;
			result.m02 = xz + wy;
			result.m03 = 0.f;

			result.m10 = xy + wz;
			result.m11 = 1.f - (xx + zz);
			result.m12 = yz - wx;
			result.m13 = 0.f;

			result.m20 = xz - wy;
			result.m21 = yz + wx;
			result.m22 = 1.f - (xx + yy);
			result.m23 = 0.f;

			result.m30 = 0.f;
			result.m31 = 0.f;
			result.m32 = 0.f;
			result.m33 = 1.f;

			return result;
		}

		static inline constexpr Matrix4x4 Scale(const Vector3& scale)
		{
			Matrix4x4 result;

			result.m00 = scale.x;
			result.m01 = 0.f;
			result.m02 = 0.f;
			result.m03 = 0.f;

			result.m10 = 0.f;
			result.m11 = scale.y;
			result.m12 = 0.f;
			result.m13 = 0.f;

			result.m20 = 0.f;
			result.m21 = 0.f;
			result.m22 = scale.z;
			result.m23 = 0.f;

			result.m30 = 0.f;
			result.m31 = 0.f;
			result.m32 = 0.f;
			result.m33 = 1.f;

			return result;
		}

		static inline constexpr Matrix4x4 TRS(const Vector3& pos, const Quaternion& q, const Vector3& s)
		{
			Matrix4x4 result = Rotate(q);

			result.m00 *= s.x;
			result.m01 *= s.x;
			result.m02 *= s.x;

			result.m10 *= s.y;
			result.m11 *= s.y;
			result.m12 *= s.y;

			result.m20 *= s.z;
			result.m21 *= s.z;
			result.m22 *= s.z;

			result.m03 = pos.x;
			result.m13 = pos.y;
			result.m23 = pos.z;

			return result;
		}

		inline constexpr bool Equals(const Matrix4x4& other, float tolerance = LOOSE_EPSILON) const
		{
			for (int i = 0; i < 16; i++)
			{
				if (Abs(data[i] - other.data[i]) > tolerance)
				{
					return false;
				}
			}

			return true;
		}

		inline constexpr float Determinant() const
		{
			float minor0 = m11 * (m22 * m33 - m23 * m32) - m12 * (m21 * m33 - m23 * m31) + m13 * (m21 * m32 - m22 * m31);
			float minor1 = m10 * (m22 * m33 - m23 * m32) - m12 * (m20 * m33 - m23 * m30) + m13 * (m20 * m32 - m22 * m30);
			float minor2 = m10 * (m21 * m33 - m23 * m31) - m11 * (m20 * m33 - m23 * m30) + m13 * (m20 * m31 - m21 * m30);
			float minor3 = m10 * (m21 * m32 - m22 * m31) - m11 * (m20 * m32 - m22 * m30) + m12 * (m20 * m31 - m21 * m30);
			return m00 * minor0 - m01 * minor1 + m02 * minor2 - m03 * minor3;
		}

		inline constexpr Matrix4x4 Inverse() const
		{
			float det = Determinant();
			if (Abs(det) <= EPSILON)
			{
				return Matrix4x4::IDENTITY;
			}

			Matrix4x4 inv;
			float invDet = 1.f / det;

			inv.m00 = (m11 * (m22 * m33 - m23 * m32) - m12 * (m21 * m33 - m23 * m31) + m13 * (m21 * m32 - m22 * m31)) * invDet;
			inv.m01 = -(m01 * (m22 * m33 - m23 * m32) - m02 * (m21 * m33 - m23 * m31) + m03 * (m21 * m32 - m22 * m31)) * invDet;
			inv.m02 = (m01 * (m12 * m33 - m13 * m32) - m02 * (m11 * m33 - m13 * m31) + m03 * (m11 * m32 - m12 * m31)) * invDet;
			inv.m03 = -(m01 * (m12 * m23 - m13 * m22) - m02 * (m11 * m23 - m13 * m21) + m03 * (m11 * m22 - m12 * m21)) * invDet;

			inv.m10 = -(m10 * (m22 * m33 - m23 * m32) - m12 * (m20 * m33 - m23 * m30) + m13 * (m20 * m32 - m22 * m30)) * invDet;
			inv.m11 = (m00 * (m22 * m33 - m23 * m32) - m02 * (m20 * m33 - m23 * m30) + m03 * (m20 * m32 - m22 * m30)) * invDet;
			inv.m12 = -(m00 * (m12 * m33 - m13 * m32) - m02 * (m10 * m33 - m13 * m30) + m03 * (m10 * m32 - m12 * m30)) * invDet;
			inv.m13 = (m00 * (m12 * m23 - m13 * m22) - m02 * (m10 * m23 - m13 * m20) + m03 * (m10 * m22 - m12 * m20)) * invDet;

			inv.m20 = (m10 * (m21 * m33 - m23 * m31) - m11 * (m20 * m33 - m23 * m30) + m13 * (m20 * m31 - m21 * m30)) * invDet;
			inv.m21 = -(m00 * (m21 * m33 - m23 * m31) - m01 * (m20 * m33 - m23 * m30) + m03 * (m20 * m31 - m21 * m30)) * invDet;
			inv.m22 = (m00 * (m11 * m33 - m13 * m31) - m01 * (m10 * m33 - m13 * m30) + m03 * (m10 * m31 - m11 * m30)) * invDet;
			inv.m23 = -(m00 * (m11 * m23 - m13 * m21) - m01 * (m10 * m23 - m13 * m20) + m03 * (m10 * m21 - m11 * m20)) * invDet;

			inv.m30 = -(m10 * (m21 * m32 - m22 * m31) - m11 * (m20 * m32 - m22 * m30) + m12 * (m20 * m31 - m21 * m30)) * invDet;
			inv.m31 = (m00 * (m21 * m32 - m22 * m31) - m01 * (m20 * m32 - m22 * m30) + m02 * (m20 * m31 - m21 * m30)) * invDet;
			inv.m32 = -(m00 * (m11 * m32 - m12 * m31) - m01 * (m10 * m32 - m12 * m30) + m02 * (m10 * m31 - m11 * m30)) * invDet;
			inv.m33 = (m00 * (m11 * m22 - m12 * m21) - m01 * (m10 * m22 - m12 * m20) + m02 * (m10 * m21 - m11 * m20)) * invDet;

			return inv;
		}

		inline constexpr bool IsIdentity(float tolerance = EPSILON) const
		{
			return
				Abs(m00 - 1.f) <= tolerance &&
				Abs(m01) <= tolerance &&
				Abs(m02) <= tolerance &&
				Abs(m03) <= tolerance &&

				Abs(m10) <= tolerance &&
				Abs(m11 - 1.f) <= tolerance &&
				Abs(m12) <= tolerance &&
				Abs(m13) <= tolerance &&

				Abs(m20) <= tolerance &&
				Abs(m21) <= tolerance &&
				Abs(m22 - 1.f) <= tolerance &&
				Abs(m23) <= tolerance &&

				Abs(m30) <= tolerance &&
				Abs(m31) <= tolerance &&
				Abs(m32) <= tolerance &&
				Abs(m33 - 1.f) <= tolerance;
		}

		inline constexpr Matrix4x4 Transpose() const
		{
			Matrix4x4 result;

			for (size_t y = 0; y < 4; y++)
			{
				for (size_t x = 0; x < 4; x++)
				{
					result.data[y + x * 4] = data[x + y * 4];
				}
			}

			return result;
		}

		inline bool Decompose(Vector3& position, Quaternion& rotation, Vector3& scale) const noexcept
		{
			position = GetPosition();
			rotation = GetRotation();
			scale = GetScale();
			if (scale.x <= EPSILON || scale.y <= EPSILON || scale.z <= EPSILON)
			{
				return false;
			}

			return true;
		}

		inline constexpr Vector3 MultiplyPoint(const Vector3& point) const
		{
			float w = m30 * point.x + m31 * point.y + m32 * point.z + m33;

			if (Abs(w - 1.f) < EPSILON)
			{
				return Vector3(
					m00 * point.x + m01 * point.y + m02 * point.z + m03,
					m10 * point.x + m11 * point.y + m12 * point.z + m13,
					m20 * point.x + m21 * point.y + m22 * point.z + m23
				);
			}
			else if (Abs(w) > EPSILON)
			{
				float invW = 1.f / w;
				return Vector3(
					(m00 * point.x + m01 * point.y + m02 * point.z + m03) * invW,
					(m10 * point.x + m11 * point.y + m12 * point.z + m13) * invW,
					(m20 * point.x + m21 * point.y + m22 * point.z + m23) * invW
				);
			}
			else
			{
				return MultiplyPoint3x4(point);
			}
		}

		inline constexpr Vector3 MultiplyPoint3x4(const Vector3& point) const
		{
			return Vector3(
				m00 * point.x + m01 * point.y + m02 * point.z + m03,
				m10 * point.x + m11 * point.y + m12 * point.z + m13,
				m20 * point.x + m21 * point.y + m22 * point.z + m23
			);
		}

		inline constexpr Vector3 MultiplyVector(const Vector3& v) const
		{
			return Vector3(
				m00 * v.x + m01 * v.y + m02 * v.z,
				m10 * v.x + m11 * v.y + m12 * v.z,
				m20 * v.x + m21 * v.y + m22 * v.z
			);
		}

		inline constexpr Vector3 GetPosition() const
		{
			return Vector3(m03, m13, m23);
		}

		inline Vector3 GetScale() const
		{
			return Vector3(
				Vector3(m00, m10, m20).Length(),
				Vector3(m01, m11, m21).Length(),
				Vector3(m02, m12, m22).Length()
			);
		}

		inline Quaternion GetRotation() const
		{
			Vector3 scale = GetScale();
			if (scale.x <= EPSILON || scale.y <= EPSILON || scale.z <= EPSILON)
			{
				return Quaternion::IDENTITY;
			}

			Matrix4x4 rotMatrix;

			rotMatrix.m00 = m00 / scale.x;
			rotMatrix.m01 = m01 / scale.y;
			rotMatrix.m02 = m02 / scale.z;
			rotMatrix.m03 = 0.f;

			rotMatrix.m10 = m10 / scale.x;
			rotMatrix.m11 = m11 / scale.y;
			rotMatrix.m12 = m12 / scale.z;
			rotMatrix.m13 = 0.f;

			rotMatrix.m20 = m20 / scale.x;
			rotMatrix.m21 = m21 / scale.y;
			rotMatrix.m22 = m22 / scale.z;
			rotMatrix.m23 = 0.f;

			rotMatrix.m30 = 0.f;
			rotMatrix.m31 = 0.f;
			rotMatrix.m32 = 0.f;
			rotMatrix.m33 = 1.f;

			Quaternion q;
			float trace = rotMatrix.m00 + rotMatrix.m11 + rotMatrix.m22;

			if (trace > 0.f)
			{
				float s = 0.5f / std::sqrt(trace + 1.f);
				q.w = 0.25f / s;
				q.x = (rotMatrix.m21 - rotMatrix.m12) * s;
				q.y = (rotMatrix.m02 - rotMatrix.m20) * s;
				q.z = (rotMatrix.m10 - rotMatrix.m01) * s;
			}
			else
			{
				if (rotMatrix.m00 > rotMatrix.m11 && rotMatrix.m00 > rotMatrix.m22)
				{
					float s = 2.f * std::sqrt(1.f + rotMatrix.m00 - rotMatrix.m11 - rotMatrix.m22);
					q.w = (rotMatrix.m21 - rotMatrix.m12) / s;
					q.x = 0.25f * s;
					q.y = (rotMatrix.m01 + rotMatrix.m10) / s;
					q.z = (rotMatrix.m02 + rotMatrix.m20) / s;
				}
				else if (rotMatrix.m11 > rotMatrix.m22)
				{
					float s = 2.f * std::sqrt(1.f + rotMatrix.m11 - rotMatrix.m00 - rotMatrix.m22);
					q.w = (rotMatrix.m02 - rotMatrix.m20) / s;
					q.x = (rotMatrix.m01 + rotMatrix.m10) / s;
					q.y = 0.25f * s;
					q.z = (rotMatrix.m12 + rotMatrix.m21) / s;
				}
				else
				{
					float s = 2.f * std::sqrt(1.f + rotMatrix.m22 - rotMatrix.m00 - rotMatrix.m11);
					q.w = (rotMatrix.m10 - rotMatrix.m01) / s;
					q.x = (rotMatrix.m02 + rotMatrix.m20) / s;
					q.y = (rotMatrix.m12 + rotMatrix.m21) / s;
					q.z = 0.25f * s;
				}
			}

			return q.Normalized();
		}

		inline constexpr Vector4 GetRow(size_t index) const
		{
			switch (index)
			{
			case 0: return Vector4(m00, m01, m02, m03);
			case 1: return Vector4(m10, m11, m12, m13);
			case 2: return Vector4(m20, m21, m22, m23);
			case 3: return Vector4(m30, m31, m32, m33);
			default: return Vector4::ZERO;
			}
		}

		inline constexpr Vector4 GetColumn(size_t index) const
		{
			switch (index)
			{
			case 0: return Vector4(m00, m10, m20, m30);
			case 1: return Vector4(m01, m11, m21, m31);
			case 2: return Vector4(m02, m12, m22, m32);
			case 3: return Vector4(m03, m13, m23, m33);
			default: return Vector4::ZERO;
			}
		}

		inline constexpr void SetRow(size_t index, const Vector4& row)
		{
			switch (index)
			{
			case 0: m00 = row.x; m01 = row.y; m02 = row.z; m03 = row.w; break;
			case 1: m10 = row.x; m11 = row.y; m12 = row.z; m13 = row.w; break;
			case 2: m20 = row.x; m21 = row.y; m22 = row.z; m23 = row.w; break;
			case 3: m30 = row.x; m31 = row.y; m32 = row.z; m33 = row.w; break;
			default: break;
			}
		}

		inline constexpr void SetColumn(size_t index, const Vector4& column)
		{
			switch (index)
			{
			case 0: m00 = column.x; m10 = column.y; m20 = column.z; m30 = column.w; break;
			case 1: m01 = column.x; m11 = column.y; m21 = column.z; m31 = column.w; break;
			case 2: m02 = column.x; m12 = column.y; m22 = column.z; m32 = column.w; break;
			case 3: m03 = column.x; m13 = column.y; m23 = column.z; m33 = column.w; break;
			default: break;
			}
		}

		inline constexpr void SetTRS(const Vector3& pos, const Quaternion& q, const Vector3& s)
		{
			float x2 = q.x * 2.f;
			float y2 = q.y * 2.f;
			float z2 = q.z * 2.f;
			float xx2 = q.x * x2;
			float yy2 = q.y * y2;
			float zz2 = q.z * z2;
			float xy2 = q.x * y2;
			float xz2 = q.x * z2;
			float yz2 = q.y * z2;
			float wx2 = q.w * x2;
			float wy2 = q.w * y2;
			float wz2 = q.w * z2;

			m00 = (1.f - (yy2 + zz2)) * s.x;
			m01 = (xy2 - wz2) * s.y;
			m02 = (xz2 + wy2) * s.z;
			m03 = pos.x;

			m10 = (xy2 + wz2) * s.x;
			m11 = (1.f - (xx2 + zz2)) * s.y;
			m12 = (yz2 - wx2) * s.z;
			m13 = pos.y;

			m20 = (xz2 - wy2) * s.x;
			m21 = (yz2 + wx2) * s.y;
			m22 = (1.f - (xx2 + yy2)) * s.z;
			m23 = pos.z;

			m30 = 0.f;
			m31 = 0.f;
			m32 = 0.f;
			m33 = 1.f;
		}

		inline bool IsValidTRS(float tolerance = EPSILON) const
		{
			if (Abs(m30) > tolerance || Abs(m31) > tolerance || Abs(m32) > tolerance || Abs(m33 - 1.f) > tolerance)
			{
				return false;
			}

			Vector3 c0(m00, m10, m20);
			Vector3 c1(m01, m11, m21);
			Vector3 c2(m02, m12, m22);

			if (c0.LengthSquared() <= tolerance || c1.LengthSquared() <= tolerance || c2.LengthSquared() <= tolerance)
			{
				return false;
			}

			c0.Normalize();
			c1.Normalize();
			c2.Normalize();

			float dot01 = Vector3::Dot(c0, c1);
			float dot02 = Vector3::Dot(c0, c2);
			float dot12 = Vector3::Dot(c1, c2);

			return Abs(dot01) <= tolerance && Abs(dot02) <= tolerance && Abs(dot12) <= tolerance;
		}

		inline std::string ToString() const
		{
			return std::format(
				"{:.5f}, {:.5f}, {:.5f}, {:.5f}\n"
				"{:.5f}, {:.5f}, {:.5f}, {:.5f}\n"
				"{:.5f}, {:.5f}, {:.5f}, {:.5f}\n"
				"{:.5f}, {:.5f}, {:.5f}, {:.5f}",
				m00, m01, m02, m03,
				m10, m11, m12, m13,
				m20, m21, m22, m23,
				m30, m31, m32, m33
			);
		}
	};

	inline const Matrix4x4 Matrix4x4::ZERO = Matrix4x4(Vector4::ZERO, Vector4::ZERO, Vector4::ZERO, Vector4::ZERO);
	inline const Matrix4x4 Matrix4x4::IDENTITY = Matrix4x4(Vector4(1.f, 0.f, 0.f, 0.f), Vector4(0.f, 1.f, 0.f, 0.f), Vector4(0.f, 0.f, 1.f, 0.f), Vector4(0.f, 0.f, 0.f, 1.f));
}

