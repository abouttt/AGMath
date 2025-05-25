#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <format>
#include <stdexcept>
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

		constexpr Matrix4x4(
			float m00, float m10, float m20, float m30,
			float m01, float m11, float m21, float m31,
			float m02, float m12, float m22, float m32,
			float m03, float m13, float m23, float m33)
			: m00(m00), m10(m10), m20(m20), m30(m30)
			, m01(m01), m11(m11), m21(m21), m31(m31)
			, m02(m02), m12(m12), m22(m22), m32(m32)
			, m03(m03), m13(m13), m23(m23), m33(m33)
		{
		}

	public:

		float operator()(size_t row, size_t column) const
		{
			if (row >= 4 || column >= 4)
			{
				throw std::out_of_range("Invalid Matrix4x4 index!");
			}

			return m[row + column * 4];
		}

		float& operator()(size_t row, size_t column)
		{
			if (row >= 4 || column >= 4)
			{
				throw std::out_of_range("Invalid Matrix4x4 index!");
			}

			return m[row + column * 4];
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

		Matrix4x4 operator*(const Matrix4x4& other) const
		{
			Matrix4x4 result = ZERO;

			for (size_t r = 0; r < 4; r++)
			{
				for (size_t c = 0; c < 4; c++)
				{
					for (size_t k = 0; k < 4; k++)
					{
						result(r, c) += (*this)(r, k) * other(k, c);
					}
				}
			}

			return result;
		}

		constexpr Matrix4x4& operator*=(float scalar)
		{
			for (float& e : m)
			{
				e *= scalar;
			}

			return *this;
		}

		Matrix4x4& operator*=(const Matrix4x4& other)
		{
			*this = *this * other;
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

		friend constexpr Matrix4x4 operator*(float scalar, const Matrix4x4& mat)
		{
			return mat * scalar;
		}

	public:

		constexpr float Determinant() const
		{
			float c00 = (m11 * (m22 * m33 - m23 * m32) - m12 * (m21 * m33 - m23 * m31) + m13 * (m21 * m32 - m22 * m31));
			float c01 = -(m10 * (m22 * m33 - m23 * m32) - m12 * (m20 * m33 - m23 * m30) + m13 * (m20 * m32 - m22 * m30));
			float c02 = (m10 * (m21 * m33 - m23 * m31) - m11 * (m20 * m33 - m23 * m30) + m13 * (m20 * m31 - m21 * m30));
			float c03 = -(m10 * (m21 * m32 - m22 * m31) - m11 * (m20 * m32 - m22 * m30) + m12 * (m20 * m31 - m21 * m30));
			return m00 * c00 + m01 * c01 + m02 * c02 + m03 * c03;
		}

		constexpr Matrix4x4 Inverse() const
		{
			Matrix4x4 adj;

			adj.m00 = (m11 * (m22 * m33 - m23 * m32) - m12 * (m21 * m33 - m23 * m31) + m13 * (m21 * m32 - m22 * m31));
			adj.m01 = -(m01 * (m22 * m33 - m23 * m32) - m02 * (m21 * m33 - m23 * m31) + m03 * (m21 * m32 - m22 * m31));
			adj.m02 = (m01 * (m12 * m33 - m13 * m32) - m02 * (m11 * m33 - m13 * m31) + m03 * (m11 * m32 - m12 * m31));
			adj.m03 = -(m01 * (m12 * m23 - m13 * m22) - m02 * (m11 * m23 - m13 * m21) + m03 * (m11 * m22 - m12 * m21));

			adj.m10 = -(m10 * (m22 * m33 - m23 * m32) - m12 * (m20 * m33 - m23 * m30) + m13 * (m20 * m32 - m22 * m30));
			adj.m11 = (m00 * (m22 * m33 - m23 * m32) - m02 * (m20 * m33 - m23 * m30) + m03 * (m20 * m32 - m22 * m30));
			adj.m12 = -(m00 * (m12 * m33 - m13 * m32) - m02 * (m10 * m33 - m13 * m30) + m03 * (m10 * m32 - m12 * m30));
			adj.m13 = (m00 * (m12 * m23 - m13 * m22) - m02 * (m10 * m23 - m13 * m20) + m03 * (m10 * m22 - m12 * m20));

			adj.m20 = (m10 * (m21 * m33 - m23 * m31) - m11 * (m20 * m33 - m23 * m30) + m13 * (m20 * m31 - m21 * m30));
			adj.m21 = -(m00 * (m21 * m33 - m23 * m31) - m01 * (m20 * m33 - m23 * m30) + m03 * (m20 * m31 - m21 * m30));
			adj.m22 = (m00 * (m11 * m33 - m13 * m31) - m01 * (m10 * m33 - m13 * m30) + m03 * (m10 * m31 - m11 * m30));
			adj.m23 = -(m00 * (m11 * m23 - m13 * m21) - m01 * (m10 * m23 - m13 * m20) + m03 * (m10 * m21 - m11 * m20));

			adj.m30 = -(m10 * (m21 * m32 - m22 * m31) - m11 * (m20 * m32 - m22 * m30) + m12 * (m20 * m31 - m21 * m30));
			adj.m31 = (m00 * (m21 * m32 - m22 * m31) - m01 * (m20 * m32 - m22 * m30) + m02 * (m20 * m31 - m21 * m30));
			adj.m32 = -(m00 * (m11 * m32 - m12 * m31) - m01 * (m10 * m32 - m12 * m30) + m02 * (m10 * m31 - m11 * m30));
			adj.m33 = (m00 * (m11 * m22 - m12 * m21) - m01 * (m10 * m22 - m12 * m20) + m02 * (m10 * m21 - m11 * m20));

			float det = m00 * adj.m00 + m01 * adj.m10 + m02 * adj.m20 + m03 * adj.m30;
			if (agm::IsNearlyZero(det, agm::MATRIX_EPSILON))
			{
				return ZERO;
			}

			return adj * (1.f / det);
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

		bool Decompose(Vector3& outPosition, Quaternion& outRotation, Vector3& outScale) const
		{
			outPosition = GetPosition();

			Matrix4x4 rotScaleMat = *this;
			rotScaleMat.SetColumn(3, Vector4(0.f, 0.f, 0.f, 1.f));
			outScale = rotScaleMat.GetScale();
			if (outScale.IsNearlyZero(agm::EPSILON_SQ_LENGTH))
			{
				outRotation = Quaternion::IDENTITY;
				return false;
			}

			Matrix4x4 pureRotMat = IDENTITY;
			if (!agm::IsNearlyZero(outScale.x))
			{
				pureRotMat.SetColumn(0, rotScaleMat.GetColumn(0) / outScale.x);
			}
			if (!agm::IsNearlyZero(outScale.y))
			{
				pureRotMat.SetColumn(1, rotScaleMat.GetColumn(1) / outScale.y);
			}
			if (!agm::IsNearlyZero(outScale.z))
			{
				pureRotMat.SetColumn(2, rotScaleMat.GetColumn(2) / outScale.z);
			}

			outRotation = pureRotMat.GetRotation();

			return true;
		}

		constexpr Vector4 TransformVector4(const Vector4& v) const
		{
			Vector4 result;

			result.x = m00 * v.x + m01 * v.y + m02 * v.z + m03 * v.w;
			result.y = m10 * v.x + m11 * v.y + m12 * v.z + m13 * v.w;
			result.z = m20 * v.x + m21 * v.y + m22 * v.z + m23 * v.w;
			result.w = m30 * v.x + m31 * v.y + m32 * v.z + m33 * v.w;

			return result;
		}

		constexpr Vector3 TransformVector3(const Vector3& v) const
		{
			Vector3 result;

			result.x = m00 * v.x + m01 * v.y + m02 * v.z;
			result.y = m10 * v.x + m11 * v.y + m12 * v.z;
			result.z = m20 * v.x + m21 * v.y + m22 * v.z;

			return result;
		}

		constexpr Vector3 TransformPosition(const Vector3& position) const
		{
			Vector3 result;

			result.x = m00 * position.x + m01 * position.y + m02 * position.z + m03;
			result.y = m10 * position.x + m11 * position.y + m12 * position.z + m13;
			result.z = m20 * position.x + m21 * position.y + m22 * position.z + m23;

			float wComp = m30 * position.x + m31 * position.y + m32 * position.z + m33;
			if (!agm::IsNearlyZero(wComp) && !agm::IsNearlyEqual(wComp, 1.f))
			{
				return result * (1.f / wComp);
			}

			return result;
		}

		constexpr Vector3 TransformPosition3DAffine(const Vector3& position) const
		{
			Vector3 result;

			result.x = m00 * position.x + m01 * position.y + m02 * position.z + m03;
			result.y = m10 * position.x + m11 * position.y + m12 * position.z + m13;
			result.z = m20 * position.x + m21 * position.y + m22 * position.z + m23;

			return result;
		}

		constexpr Vector3 GetPosition() const
		{
			return Vector3(m03, m13, m23);
		}

		Quaternion GetRotation() const
		{
			Matrix4x4 mat = *this;
			mat.SetPosition(Vector3::ZERO);

			Vector3 scale = mat.GetScale();
			if (scale.IsNearlyZero(agm::EPSILON_SQ_LENGTH))
			{
				return Quaternion::IDENTITY;
			}

			Matrix4x4 rotMat = IDENTITY;
			Vector3 c0 = mat.GetColumn(0).ToVector3();
			Vector3 c1 = mat.GetColumn(1).ToVector3();
			Vector3 c2 = mat.GetColumn(2).ToVector3();

			if (!IsNearlyZero(scale.x))
			{
				c0 /= scale.x;
			}
			if (!IsNearlyZero(scale.y))
			{
				c1 /= scale.y;
			}
			if (!IsNearlyZero(scale.z))
			{
				c2 /= scale.z;
			}

			if (Vector3::Dot(c0, Vector3::Cross(c1, c2)) < 0.f)
			{
				if (!IsNearlyZero(scale.z))
				{
					c2 *= -1.f;
				}
				else if (!IsNearlyZero(scale.y))
				{
					c1 *= -1.f;
				}
				else if (!IsNearlyZero(scale.x))
				{
					c0 *= -1.f;
				}
			}

			rotMat.SetColumn(0, Vector4(c0, 0.f));
			rotMat.SetColumn(1, Vector4(c1, 0.f));
			rotMat.SetColumn(2, Vector4(c2, 0.f));

			return rotMat.getRotationOnly();
		}

		Vector3 GetScale() const
		{
			return Vector3(
				GetColumn(0).ToVector3().Length(),
				GetColumn(1).ToVector3().Length(),
				GetColumn(2).ToVector3().Length()
			);
		}

		Vector4 GetRow(size_t index) const
		{
			return Vector4((*this)(index, 0), (*this)(index, 1), (*this)(index, 2), (*this)(index, 3));
		}

		Vector4 GetColumn(size_t index) const
		{
			return Vector4((*this)(0, index), (*this)(1, index), (*this)(2, index), (*this)(3, index));
		}

		void SetRow(size_t index, const Vector4& row)
		{
			(*this)(index, 0) = row.x;
			(*this)(index, 1) = row.y;
			(*this)(index, 2) = row.z;
			(*this)(index, 3) = row.w;
		}

		void SetColumn(size_t index, const Vector4& column)
		{
			(*this)(0, index) = column.x;
			(*this)(1, index) = column.y;
			(*this)(2, index) = column.z;
			(*this)(3, index) = column.w;
		}

		constexpr void SetPosition(const Vector3& position)
		{
			m03 = position.x;
			m13 = position.y;
			m23 = position.z;
		}

		constexpr void SetTRS(const Vector3& pos, const Quaternion& rot, const Vector3& scale)
		{
			Matrix4x4 rotMat = Rotate(rot);

			m00 = rotMat.m00 * scale.x;
			m01 = rotMat.m01 * scale.y;
			m02 = rotMat.m02 * scale.z;
			m03 = pos.x;

			m10 = rotMat.m10 * scale.x;
			m11 = rotMat.m11 * scale.y;
			m12 = rotMat.m12 * scale.z;
			m13 = pos.y;

			m20 = rotMat.m20 * scale.x;
			m21 = rotMat.m21 * scale.y;
			m22 = rotMat.m22 * scale.z;
			m23 = pos.z;

			m30 = 0.f;
			m31 = 0.f;
			m32 = 0.f;
			m33 = 1.f;
		}

		bool IsValidTRS() const
		{
			if (!IsNearlyZero(m30) ||
				!IsNearlyZero(m31) ||
				!IsNearlyZero(m32) ||
				!IsNearlyEqual(m33, 1.f))
			{
				return false;
			}

			Vector3 c0 = GetColumn(0).ToVector3();
			Vector3 c1 = GetColumn(1).ToVector3();
			Vector3 c2 = GetColumn(2).ToVector3();
			if (c0.IsNearlyZero(agm::EPSILON_SQ_LENGTH) ||
				c1.IsNearlyZero(agm::EPSILON_SQ_LENGTH) ||
				c2.IsNearlyZero(agm::EPSILON_SQ_LENGTH))
			{
				return false;
			}

			c0.Normalize();
			c1.Normalize();
			c2.Normalize();

			return IsNearlyZero(Vector3::Dot(c0, c1)) &&
				   IsNearlyZero(Vector3::Dot(c0, c2)) &&
				   IsNearlyZero(Vector3::Dot(c1, c2));
		}

		constexpr bool Equals(const Matrix4x4& other, float tolerance = agm::MATRIX_EPSILON) const
		{
			for (size_t i = 0; i < 16; i++)
			{
				if (!IsNearlyEqual(m[i], other.m[i], tolerance))
				{
					return false;
				}
			}

			return true;
		}

		std::string ToString() const
		{
			return std::format(
				"[{: .3f}, {: .3f}, {: .3f}, {: .3f}]\n"
				"[{: .3f}, {: .3f}, {: .3f}, {: .3f}]\n"
				"[{: .3f}, {: .3f}, {: .3f}, {: .3f}]\n"
				"[{: .3f}, {: .3f}, {: .3f}, {: .3f}]",
				m00, m01, m02, m03,
				m10, m11, m12, m13,
				m20, m21, m22, m23,
				m30, m31, m32, m33
			);
		}

	public:

		static constexpr Matrix4x4 Frustum(float left, float right, float bottom, float top, float zNear, float zFar)
		{
			Matrix4x4 result = ZERO;

			if (IsNearlyZero(right - left) ||
				IsNearlyZero(top - bottom) ||
				IsNearlyZero(zFar - zNear))
			{
				return result;
			}

			float invWidth = 1.f / (right - left);
			float invHeight = 1.f / (top - bottom);
			float invDepth = 1.f / (zFar - zNear);

			result.m00 = 2.f * zNear * invWidth;
			result.m02 = (right + left) * invWidth;

			result.m11 = 2.f * zNear * invHeight;
			result.m12 = (top + bottom) * invHeight;

			result.m22 = -(zFar + zNear) * invDepth;
			result.m23 = -(2.f * zFar * zNear) * invDepth;

			result.m32 = -1.f;

			return result;
		}

		static bool Inverse3DAffine(const Matrix4x4& input, Matrix4x4& output)
		{
			Vector3 c0 = input.GetColumn(0).ToVector3();
			Vector3 c1 = input.GetColumn(1).ToVector3();
			Vector3 c2 = input.GetColumn(2).ToVector3();

			float det3 = Vector3::Dot(c0, Vector3::Cross(c1, c2));
			if (IsNearlyZero(det3, agm::MATRIX_EPSILON))
			{
				return false;
			}

			float invDet3 = 1.f / det3;
			Vector3 pos = input.GetPosition();

			output.m00 = (c1.y * c2.z - c1.z * c2.y) * invDet3;
			output.m01 = (input.m02 * c2.y - c1.y * input.m22) * invDet3;
			output.m02 = (c1.y * input.m21 - input.m01 * c2.y) * invDet3;
			output.m03 = -(output.m00 * pos.x + output.m01 * pos.y + output.m02 * pos.z);

			output.m10 = (c1.z * c2.x - c1.x * c2.z) * invDet3;
			output.m11 = (input.m00 * c2.z - input.m02 * c2.x) * invDet3;
			output.m12 = (input.m02 * c1.x - input.m00 * c1.z) * invDet3;
			output.m13 = -(output.m10 * pos.x + output.m11 * pos.y + output.m12 * pos.z);

			output.m20 = (c1.x * c2.y - c1.y * c2.x) * invDet3;
			output.m21 = (input.m01 * c2.x - input.m00 * c2.y) * invDet3;
			output.m22 = (input.m00 * c1.y - input.m01 * c1.x) * invDet3;
			output.m23 = -(output.m20 * pos.x + output.m21 * pos.y + output.m22 * pos.z);

			output.m30 = 0.f;
			output.m31 = 0.f;
			output.m32 = 0.f;
			output.m33 = 1.f;

			return true;
		}

		static Matrix4x4 LookAt(const Vector3& eye, const Vector3& target, const Vector3& up)
		{
			Vector3 f = (target - eye).GetNormalized();
			if (f.IsNearlyZero(agm::EPSILON_SQ_LENGTH))
			{
				f = Vector3::FORWARD;
			}

			Vector3 r = Vector3::Cross(up, f).GetNormalized();
			if (r.IsNearlyZero(agm::EPSILON_SQ_LENGTH))
			{
				if (Abs(f.y) > 1.f - agm::EPSILON_DOT_ONE)
				{
					r = Vector3::Cross(Vector3::FORWARD, f);
				}
				else
				{
					r = Vector3::Cross(Vector3::UP, f);
				}

				r.Normalize();
				if (r.IsNearlyZero(agm::EPSILON_SQ_LENGTH))
				{
					r = (Abs(f.x) < 0.9f ? Vector3::RIGHT : Vector3::UP);
					r = Vector3::Cross(r, f).GetNormalized();
					if (r.IsNearlyZero(agm::EPSILON_SQ_LENGTH))
					{
						r = Vector3::RIGHT;
					}
				}
			}
			Vector3 u = Vector3::Cross(f, r);

			Matrix4x4 result;

			result.m00 = r.x;
			result.m01 = u.x;
			result.m02 = f.x;
			result.m03 = -Vector3::Dot(r, eye);

			result.m10 = r.y;
			result.m11 = u.y;
			result.m12 = f.y;
			result.m13 = -Vector3::Dot(u, eye);

			result.m20 = r.z;
			result.m21 = u.z;
			result.m22 = f.z;
			result.m23 = -Vector3::Dot(f, eye);

			result.m30 = 0.f;
			result.m31 = 0.f;
			result.m32 = 0.f;
			result.m33 = 1.f;

			return result;
		}

		static constexpr Matrix4x4 Orthographic(float left, float right, float bottom, float top, float zNear, float zFar)
		{
			Matrix4x4 result = ZERO;

			if (IsNearlyZero(right - left) ||
				IsNearlyZero(top - bottom) ||
				IsNearlyZero(zFar - zNear))
			{
				return result;
			}

			float invWidth = 1.f / (right - left);
			float invHeight = 1.f / (top - bottom);
			float invDepth = 1.f / (zFar - zNear);

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
			Matrix4x4 result = ZERO;

			if (IsNearlyZero(aspectRatio) ||
				IsNearlyZero(zFar - zNear) ||
				fovYDegrees <= 0.f || fovYDegrees >= 180.f)
			{
				return result;
			}

			float tanHalfFov = agm::Tan(fovYDegrees * DEG2RAD * 0.5f);
			float invFarMinusNear = 1.f / (zFar - zNear);

			result.m00 = 1.f / (aspectRatio * tanHalfFov);

			result.m11 = 1.f / tanHalfFov;

			result.m22 = -(zFar + zNear) * invFarMinusNear;
			result.m23 = -(2.f * zFar * zNear) * invFarMinusNear;

			result.m32 = -1.f;

			return result;
		}

		static constexpr Matrix4x4 Translate(const Vector3& v)
		{
			Matrix4x4 result = IDENTITY;

			result.m03 = v.x;
			result.m13 = v.y;
			result.m23 = v.z;

			return result;
		}

		static constexpr Matrix4x4 Rotate(const Quaternion& q)
		{
			float x2 = q.x + q.x;
			float y2 = q.y + q.y;
			float z2 = q.z + q.z;
			float xx = q.x * x2;
			float xy = q.x * y2;
			float xz = q.x * z2;
			float yy = q.y * y2;
			float yz = q.y * z2;
			float zz = q.z * z2;
			float wx = q.w * x2;
			float wy = q.w * y2;
			float wz = q.w * z2;

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

		static constexpr Matrix4x4 Scale(const Vector3& s)
		{
			Matrix4x4 result = ZERO;

			result.m00 = s.x;
			result.m11 = s.y;
			result.m22 = s.z;
			result.m33 = 1.f;

			return result;
		}

		static constexpr Matrix4x4 TRS(const Vector3& position, const Quaternion& rotation, const Vector3& scale)
		{
			Matrix4x4 result;
			result.SetTRS(position, rotation, scale);
			return result;
		}

	private:

		Quaternion getRotationOnly() const
		{
			Quaternion result;

			float trace = m00 + m11 + m22;
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
	};

	inline const Matrix4x4 Matrix4x4::ZERO = Matrix4x4(Vector4(0, 0, 0, 0), Vector4(0, 0, 0, 0), Vector4(0, 0, 0, 0), Vector4(0, 0, 0, 0));
	inline const Matrix4x4 Matrix4x4::IDENTITY = Matrix4x4(Vector4(1, 0, 0, 0), Vector4(0, 1, 0, 0), Vector4(0, 0, 1, 0), Vector4(0, 0, 0, 1));
}

