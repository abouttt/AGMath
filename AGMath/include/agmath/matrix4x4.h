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
	public:

		union
		{
			struct
			{
				float m00, m10, m20, m30;
				float m01, m11, m21, m31;
				float m02, m12, m22, m32;
				float m03, m13, m23, m33;
			};

			std::array<float, 16> m;
		};

	public:

		static const Matrix4x4 ZERO;
		static const Matrix4x4 IDENTITY;

	public:

		constexpr Matrix4x4()
			: m{}
		{
		}

		constexpr Matrix4x4(const Vector4& column0, const Vector4& column1, const Vector4& column2, const Vector4& column3)
			: m00(column0.x), m10(column0.y), m20(column0.z), m30(column0.w)
			, m01(column1.x), m11(column1.y), m21(column1.z), m31(column1.w)
			, m02(column2.x), m12(column2.y), m22(column2.z), m32(column2.w)
			, m03(column3.x), m13(column3.y), m23(column3.z), m33(column3.w)
		{
		}

	public:

		constexpr float operator()(size_t row, size_t column) const
		{
			return m[size_t(row + column * 4)];
		}

		constexpr float& operator()(size_t row, size_t column)
		{
			return m[size_t(row + column * 4)];
		}

		constexpr Matrix4x4 operator*(float scalar) const
		{
			Matrix4x4 result;

			for (size_t i = 0; i < 16; i++)
			{
				result.m[i] = m[i] * scalar;
			}

			return result;
		}

		constexpr Matrix4x4 operator*(const Matrix4x4& other) const
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

		constexpr Matrix4x4& operator*=(float scalar)
		{
			for (float& val : m)
			{
				val *= scalar;
			}

			return *this;
		}

		constexpr Matrix4x4& operator*=(const Matrix4x4& other)
		{
			*this = (*this) * other;
			return *this;
		}

		constexpr bool operator==(const Matrix4x4& other) const
		{
			for (size_t i = 0; i < 16; i++)
			{
				if (m[i] != other.m[i])
				{
					return false;
				}
			}

			return true;
		}

		constexpr bool operator!=(const Matrix4x4& other) const
		{
			return !(*this == other);
		}

	public:

		constexpr float Determinant() const
		{
			float s0 = m00 * m11 - m01 * m10;
			float s1 = m00 * m12 - m02 * m10;
			float s2 = m00 * m13 - m03 * m10;
			float s3 = m01 * m12 - m02 * m11;
			float s4 = m01 * m13 - m03 * m11;
			float s5 = m02 * m13 - m03 * m12;

			float t0 = m20 * m31 - m21 * m30;
			float t1 = m20 * m32 - m22 * m30;
			float t2 = m20 * m33 - m23 * m30;
			float t3 = m21 * m32 - m22 * m31;
			float t4 = m21 * m33 - m23 * m31;
			float t5 = m22 * m33 - m23 * m32;

			return (s0 * t5 - s1 * t4 + s2 * t3 + s3 * t2 - s4 * t1 + s5 * t0);
		}

		constexpr Matrix4x4 Inverse() const
		{
			Matrix4x4 result;

			result.m00 = (m11 * (m22 * m33 - m23 * m32) - m12 * (m21 * m33 - m23 * m31) + m13 * (m21 * m32 - m22 * m31));
			result.m01 = -(m01 * (m22 * m33 - m23 * m32) - m02 * (m21 * m33 - m23 * m31) + m03 * (m21 * m32 - m22 * m31));
			result.m02 = (m01 * (m12 * m33 - m13 * m32) - m02 * (m11 * m33 - m13 * m31) + m03 * (m11 * m32 - m12 * m31));
			result.m03 = -(m01 * (m12 * m23 - m13 * m22) - m02 * (m11 * m23 - m13 * m21) + m03 * (m11 * m22 - m12 * m21));

			result.m10 = -(m10 * (m22 * m33 - m23 * m32) - m12 * (m20 * m33 - m23 * m30) + m13 * (m20 * m32 - m22 * m30));
			result.m11 = (m00 * (m22 * m33 - m23 * m32) - m02 * (m20 * m33 - m23 * m30) + m03 * (m20 * m32 - m22 * m30));
			result.m12 = -(m00 * (m12 * m33 - m13 * m32) - m02 * (m10 * m33 - m13 * m30) + m03 * (m10 * m32 - m12 * m30));
			result.m13 = (m00 * (m12 * m23 - m13 * m22) - m02 * (m10 * m23 - m13 * m20) + m03 * (m10 * m22 - m12 * m20));

			result.m20 = (m10 * (m21 * m33 - m23 * m31) - m11 * (m20 * m33 - m23 * m30) + m13 * (m20 * m31 - m21 * m30));
			result.m21 = -(m00 * (m21 * m33 - m23 * m31) - m01 * (m20 * m33 - m23 * m30) + m03 * (m20 * m31 - m21 * m30));
			result.m22 = (m00 * (m11 * m33 - m13 * m31) - m01 * (m10 * m33 - m13 * m30) + m03 * (m10 * m31 - m11 * m30));
			result.m23 = -(m00 * (m11 * m23 - m13 * m21) - m01 * (m10 * m23 - m13 * m20) + m03 * (m10 * m21 - m11 * m20));

			result.m30 = -(m10 * (m21 * m32 - m22 * m31) - m11 * (m20 * m32 - m22 * m30) + m12 * (m20 * m31 - m21 * m30));
			result.m31 = (m00 * (m21 * m32 - m22 * m31) - m01 * (m20 * m32 - m22 * m30) + m02 * (m20 * m31 - m21 * m30));
			result.m32 = -(m00 * (m11 * m32 - m12 * m31) - m01 * (m10 * m32 - m12 * m30) + m02 * (m10 * m31 - m11 * m30));
			result.m33 = (m00 * (m11 * m22 - m12 * m21) - m01 * (m10 * m22 - m12 * m20) + m02 * (m10 * m21 - m11 * m20));

			float det = m00 * result.m00 + m01 * result.m10 + m02 * result.m20 + m03 * result.m30;
			if (Abs(det) <= EPSILON)
			{
				return Matrix4x4::ZERO;
			}

			result *= (1.f / det);
			return result;
		}

		constexpr Matrix4x4 Transpose() const
		{
			Matrix4x4 result;

			result.m00 = m00;
			result.m01 = m10;
			result.m02 = m20;
			result.m03 = m30;

			result.m10 = m01;
			result.m11 = m11;
			result.m12 = m21;
			result.m13 = m31;

			result.m20 = m02;
			result.m21 = m12;
			result.m22 = m22;
			result.m23 = m32;

			result.m30 = m03;
			result.m31 = m13;
			result.m32 = m23;
			result.m33 = m33;

			return result;
		}

		bool Decompose(Vector3& outPosition, Quaternion& outRotation, Vector3& outScale) const noexcept
		{
			outPosition = GetPosition();
			outScale = GetScale();

			if (Abs(outScale.x) <= EPSILON || Abs(outScale.y) <= EPSILON || Abs(outScale.z) <= EPSILON)
			{
				outRotation = Quaternion::IDENTITY;
				return false;
			}

			Matrix4x4 rotationMatrix;

			rotationMatrix.SetColumn(0, GetColumn(0) / outScale.x);
			rotationMatrix.SetColumn(1, GetColumn(1) / outScale.y);
			rotationMatrix.SetColumn(2, GetColumn(2) / outScale.z);
			rotationMatrix.m33 = 1.f;

			outRotation = rotationMatrix.getRotationOnly();

			return true;
		}

		constexpr Vector4 TransformVector4(const Vector4& v) const
		{
			return Vector4(
				m00 * v.x + m01 * v.y + m02 * v.z + m03 * v.w,
				m10 * v.x + m11 * v.y + m12 * v.z + m13 * v.w,
				m20 * v.x + m21 * v.y + m22 * v.z + m23 * v.w,
				m30 * v.x + m31 * v.y + m32 * v.z + m33 * v.w
			);
		}

		constexpr Vector3 TransformVector3(const Vector3& v) const
		{
			return Vector3(
				m00 * v.x + m01 * v.y + m02 * v.z,
				m10 * v.x + m11 * v.y + m12 * v.z,
				m20 * v.x + m21 * v.y + m22 * v.z
			);
		}

		constexpr Vector3 TransformPosition(const Vector3& position) const
		{
			Vector3 result;

			result.x = m00 * position.x + m01 * position.y + m02 * position.z + m03;
			result.y = m10 * position.x + m11 * position.y + m12 * position.z + m13;
			result.z = m20 * position.x + m21 * position.y + m22 * position.z + m23;

			float w = m30 * position.x + m31 * position.y + m32 * position.z + m33;
			if (Abs(w) > EPSILON && Abs(w - 1.f) > EPSILON)
			{
				float invW = 1.f / w;
				return result * invW;
			}

			return result;
		}

		constexpr Vector3 TransformPosition3DAffine(const Vector3& position) const
		{
			return Vector3(
				m00 * position.x + m01 * position.y + m02 * position.z + m03,
				m10 * position.x + m11 * position.y + m12 * position.z + m13,
				m20 * position.x + m21 * position.y + m22 * position.z + m23
			);
		}

		constexpr Vector3 GetPosition() const
		{
			return Vector3(m03, m13, m23);
		}

		Quaternion GetRotation() const
		{
			Vector3 scale = GetScale();
			if (Abs(scale.x) <= EPSILON || Abs(scale.y) <= EPSILON || Abs(scale.z) <= EPSILON)
			{
				return Quaternion::IDENTITY;
			}

			Vector3 col0 = Vector3(m00, m10, m20) / scale.x;
			Vector3 col1 = Vector3(m01, m11, m21) / scale.y;
			Vector3 col2 = Vector3(m02, m12, m22) / scale.z;

			float det = Vector3::Dot(col0, Vector3::Cross(col1, col2));
			if (det < 0.f)
			{
				scale.z *= -1.f;
				col2 *= -1.f;
			}

			Matrix4x4 rotationMatrix;

			rotationMatrix.SetColumn(0, Vector4(col0.x, col0.y, col0.z, 0.f));
			rotationMatrix.SetColumn(1, Vector4(col1.x, col1.y, col1.z, 0.f));
			rotationMatrix.SetColumn(2, Vector4(col2.x, col2.y, col2.z, 0.f));
			rotationMatrix.m33 = 1.f;

			return rotationMatrix.getRotationOnly();
		}

		Vector3 GetScale() const
		{
			Vector4 col0 = GetColumn(0);
			Vector4 col1 = GetColumn(1);
			Vector4 col2 = GetColumn(2);

			return Vector3(
				Vector3(col0.x, col0.y, col0.z).Length(),
				Vector3(col1.x, col1.y, col1.z).Length(),
				Vector3(col2.x, col2.y, col2.z).Length()
			);
		}

		constexpr Vector4 GetRow(size_t index) const
		{
			return Vector4((*this)(index, 0), (*this)(index, 1), (*this)(index, 2), (*this)(index, 3));
		}

		constexpr Vector4 GetColumn(size_t index) const
		{
			return Vector4((*this)(0, index), (*this)(1, index), (*this)(2, index), (*this)(3, index));
		}

		constexpr void SetRow(size_t index, const Vector4& row)
		{
			(*this)(index, 0) = row.x;
			(*this)(index, 1) = row.y;
			(*this)(index, 2) = row.z;
			(*this)(index, 3) = row.w;
		}

		constexpr void SetColumn(size_t index, const Vector4& column)
		{
			(*this)(0, index) = column.x;
			(*this)(1, index) = column.y;
			(*this)(2, index) = column.z;
			(*this)(3, index) = column.w;
		}

		constexpr void SetTRS(const Vector3& pos, const Quaternion& q, const Vector3& s)
		{
			float x = q.x;
			float y = q.y;
			float z = q.z;
			float w = q.w;
			float x2 = x + x;
			float y2 = y + y;
			float z2 = z + z;
			float xx = x * x2;
			float xy = x * y2;
			float xz = x * z2;
			float yy = y * y2;
			float yz = y * z2;
			float zz = z * z2;
			float wx = w * x2;
			float wy = w * y2;
			float wz = w * z2;

			m00 = (1.f - (yy + zz)) * s.x;
			m01 = (xy - wz) * s.y;
			m02 = (xz + wy) * s.z;
			m03 = pos.x;

			m10 = (xy + wz) * s.x;
			m11 = (1.f - (xx + zz)) * s.y;
			m12 = (yz - wx) * s.z;
			m13 = pos.y;

			m20 = (xz - wy) * s.x;
			m21 = (yz + wx) * s.y;
			m22 = (1.f - (xx + yy)) * s.z;
			m23 = pos.z;

			m30 = 0.f;
			m31 = 0.f;
			m32 = 0.f;
			m33 = 1.f;
		}

		bool IsValidTRS() const
		{
			if (Abs(m30) > EPSILON || Abs(m31) > EPSILON || Abs(m32) > EPSILON || Abs(m33 - 1.f) > EPSILON)
			{
				return false;
			}

			Vector4 c0_4 = GetColumn(0);
			Vector4 c1_4 = GetColumn(1);
			Vector4 c2_4 = GetColumn(2);

			Vector3 c0(c0_4.x, c0_4.y, c0_4.z);
			Vector3 c1(c1_4.x, c1_4.y, c1_4.z);
			Vector3 c2(c2_4.x, c2_4.y, c2_4.z);

			if (c0.LengthSquared() <= EPSILON * EPSILON || c1.LengthSquared() <= EPSILON * EPSILON || c2.LengthSquared() <= EPSILON * EPSILON)
			{
				return false;
			}

			c0.Normalize();
			c1.Normalize();
			c2.Normalize();

			float dot01 = Vector3::Dot(c0, c1);
			float dot02 = Vector3::Dot(c0, c2);
			float dot12 = Vector3::Dot(c1, c2);

			return Abs(dot01) <= LOOSE_EPSILON && Abs(dot02) <= LOOSE_EPSILON && Abs(dot12) <= LOOSE_EPSILON;
		}

		constexpr bool IsIdentity(float tolerance = EPSILON) const
		{
			return Equals(IDENTITY, tolerance);
		}

		constexpr bool Equals(const Matrix4x4& other, float tolerance = LOOSE_EPSILON) const
		{
			for (size_t i = 0; i < 16; i++)
			{
				if (Abs(m[i] - other.m[i]) > tolerance)
				{
					return false;
				}
			}

			return true;
		}

		std::string ToString() const
		{
			return std::format(
				"{:.5f}, {:.5f}, {:.5f}, {:.5f}\n" "{:.5f}, {:.5f}, {:.5f}, {:.5f}\n"
				"{:.5f}, {:.5f}, {:.5f}, {:.5f}\n" "{:.5f}, {:.5f}, {:.5f}, {:.5f}",
				m00, m01, m02, m03, m10, m11, m12, m13,
				m20, m21, m22, m23, m30, m31, m32, m33);
		}

	public:

		static constexpr Matrix4x4 Frustum(float left, float right, float bottom, float top, float zNear, float zFar)
		{
			float invWidth = 1.f / (right - left);
			float invHeight = 1.f / (top - bottom);
			float invDepth = 1.f / (zFar - zNear);

			Matrix4x4 result = ZERO;

			result.m00 = 2.f * zNear * invWidth;
			result.m02 = (right + left) * invWidth;

			result.m11 = 2.f * zNear * invHeight;
			result.m12 = (top + bottom) * invHeight;

			result.m22 = -(zFar + zNear) * invDepth;
			result.m23 = -(2.f * zFar * zNear) * invDepth;

			result.m32 = -1.f;

			return result;
		}

		static constexpr bool Inverse3DAffine(const Matrix4x4& input, Matrix4x4& output)
		{
			float det3 =
				input.m00 * (input.m11 * input.m22 - input.m12 * input.m21) -
				input.m01 * (input.m10 * input.m22 - input.m12 * input.m20) +
				input.m02 * (input.m10 * input.m21 - input.m11 * input.m20);
			if (Abs(det3) <= EPSILON)
			{
				return false;
			}

			float invDet3 = 1.f / det3;

			output.m00 = (input.m11 * input.m22 - input.m12 * input.m21) * invDet3;
			output.m01 = (input.m02 * input.m21 - input.m01 * input.m22) * invDet3;
			output.m02 = (input.m01 * input.m12 - input.m02 * input.m11) * invDet3;
			output.m03 = -(output.m00 * input.m03 + output.m01 * input.m13 + output.m02 * input.m23);

			output.m10 = (input.m12 * input.m20 - input.m10 * input.m22) * invDet3;
			output.m11 = (input.m00 * input.m22 - input.m02 * input.m20) * invDet3;
			output.m12 = (input.m02 * input.m10 - input.m00 * input.m12) * invDet3;
			output.m13 = -(output.m10 * input.m03 + output.m11 * input.m13 + output.m12 * input.m23);

			output.m20 = (input.m10 * input.m21 - input.m11 * input.m20) * invDet3;
			output.m21 = (input.m01 * input.m20 - input.m00 * input.m21) * invDet3;
			output.m22 = (input.m00 * input.m11 - input.m01 * input.m10) * invDet3;
			output.m23 = -(output.m20 * input.m03 + output.m21 * input.m13 + output.m22 * input.m23);

			output.m30 = 0.f;
			output.m31 = 0.f;
			output.m32 = 0.f;
			output.m33 = 1.f;

			return true;
		}

		static Matrix4x4 LookAt(const Vector3& from, const Vector3& to, const Vector3& up)
		{
			Vector3 zAxis = (to - from).GetNormalized();
			if (zAxis.LengthSquared() <= EPSILON * EPSILON)
			{
				zAxis = Vector3::FORWARD;
			}

			Vector3 xAxis = Vector3::Cross(up, zAxis).GetNormalized();
			if (xAxis.LengthSquared() <= EPSILON * EPSILON)
			{
				if (Abs(zAxis.y - 1.f) <= EPSILON || Abs(zAxis.y + 1.f) <= EPSILON)
				{
					xAxis = Vector3::Cross(Vector3::RIGHT, zAxis).GetNormalized();
					if (xAxis.LengthSquared() <= EPSILON * EPSILON)
					{
						xAxis = Vector3::Cross(Vector3::UP, zAxis).GetNormalized();
					}
				}
				else
				{
					xAxis = Vector3::Cross(Vector3::UP, zAxis).GetNormalized();
				}

				if (xAxis.LengthSquared() <= EPSILON * EPSILON)
				{
					xAxis = Vector3::RIGHT;
				}
			}

			Vector3 yAxis = Vector3::Cross(zAxis, xAxis).GetNormalized();

			Matrix4x4 result = ZERO;

			result.m00 = xAxis.x;
			result.m01 = xAxis.y;
			result.m02 = xAxis.z;
			result.m03 = -Vector3::Dot(xAxis, from);

			result.m10 = yAxis.x;
			result.m11 = yAxis.y;
			result.m12 = yAxis.z;
			result.m13 = -Vector3::Dot(yAxis, from);

			result.m20 = zAxis.x;
			result.m21 = zAxis.y;
			result.m22 = zAxis.z;
			result.m23 = -Vector3::Dot(zAxis, from);

			result.m33 = 1.f;

			return result;
		}

		static constexpr Matrix4x4 Orthographic(float left, float right, float bottom, float top, float zNear, float zFar)
		{
			float invWidth = 1.f / (right - left);
			float invHeight = 1.f / (top - bottom);
			float invDepth = 1.f / (zFar - zNear);

			Matrix4x4 result = ZERO;

			result.m00 = 2.f * invWidth;
			result.m03 = -(right + left) * invWidth;

			result.m11 = 2.f * invHeight;
			result.m13 = -(top + bottom) * invHeight;

			result.m22 = -2.f * invDepth;
			result.m23 = -(zFar + zNear) * invDepth;

			result.m33 = 1.f;

			return result;
		}

		static Matrix4x4 Perspective(float fovYDegrees, float aspectRatio, float zNear, float zFar)
		{
			float tanHalfFovY = std::tan(fovYDegrees * DEG2RAD * 0.5f);
			float invDepth = 1.f / (zFar - zNear);

			Matrix4x4 result = ZERO;

			result.m00 = 1.f / (aspectRatio * tanHalfFovY);
			result.m11 = 1.f / tanHalfFovY;
			result.m22 = -(zFar + zNear) * invDepth;
			result.m23 = -(2.f * zFar * zNear) * invDepth;
			result.m32 = -1.f;

			return result;
		}

		static Matrix4x4 Translate(const Vector3& v)
		{
			Matrix4x4 result = IDENTITY;

			result.m03 = v.x;
			result.m13 = v.y;
			result.m23 = v.z;

			return result;
		}

		static constexpr Matrix4x4 Rotate(const Quaternion& r)
		{
			float x = r.x;
			float y = r.y;
			float z = r.z;
			float w = r.w;
			float x2 = x + x;
			float y2 = y + y;
			float z2 = z + z;
			float xx = x * x2;
			float xy = x * y2;
			float xz = x * z2;
			float yy = y * y2;
			float yz = y * z2;
			float zz = z * z2;
			float wx = w * x2;
			float wy = w * y2;
			float wz = w * z2;

			Matrix4x4 result = ZERO;

			result.m00 = 1.f - (yy + zz);
			result.m01 = xy - wz;
			result.m02 = xz + wy;

			result.m10 = xy + wz;
			result.m11 = 1.f - (xx + zz);
			result.m12 = yz - wx;

			result.m20 = xz - wy;
			result.m21 = yz + wx;
			result.m22 = 1.f - (xx + yy);

			result.m33 = 1.f;

			return result;
		}

		static constexpr Matrix4x4 Scale(const Vector3& scale)
		{
			Matrix4x4 result = ZERO;

			result.m00 = scale.x;
			result.m11 = scale.y;
			result.m22 = scale.z;
			result.m33 = 1.f;

			return result;
		}

		static constexpr Matrix4x4 TRS(const Vector3& pos, const Quaternion& q, const Vector3& s)
		{
			Matrix4x4 result;
			result.SetTRS(pos, q, s);
			return result;
		}

	private:

		Quaternion getRotationOnly() const
		{
			Quaternion q;
			float trace = m00 + m11 + m22;

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
	};

	inline const Matrix4x4 Matrix4x4::ZERO = Matrix4x4();
	inline const Matrix4x4 Matrix4x4::IDENTITY = Matrix4x4(Vector4(1.f, 0.f, 0.f, 0.f), Vector4(0.f, 1.f, 0.f, 0.f), Vector4(0.f, 0.f, 1.f, 0.f), Vector4(0.f, 0.f, 0.f, 1.f));
}

