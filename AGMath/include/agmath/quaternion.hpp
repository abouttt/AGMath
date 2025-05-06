#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <format>
#include <string>

#include "concepts.hpp"
#include "constants.hpp"
#include "utilities.hpp"
#include "vector3.hpp"

namespace agm
{
	template<FloatingPoint T>
	struct Quaternion
	{
		// Static Member Variables
		static const Quaternion identity;

		// Member Variables
		union
		{
			struct
			{
				T x, y, z, w;
			};

			std::array<T, 4> data;
		};

		// Constructors
		constexpr Quaternion()
			: x(T(0))
			, y(T(0))
			, z(T(0))
			, w(T(0))
		{
		}

		constexpr Quaternion(T x, T y, T z, T w)
			: x(x)
			, y(y)
			, z(z)
			, w(w)
		{
		}

		// Operators
		inline constexpr T operator[](size_t index) const
		{
			return data[index];
		}

		inline constexpr T& operator[](size_t index)
		{
			return data[index];
		}

		inline constexpr Quaternion operator*(T scalar) const
		{
			return Quaternion(x * scalar, y * scalar, z * scalar, w * scalar);
		}

		inline constexpr Quaternion operator/(T scalar) const
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

		inline constexpr Vector3<T> operator*(const Vector3<T>& point) const
		{
			Vector3<T> qv(x, y, z);
			Vector3<T> uv = Vector3<T>::cross(qv, point);
			Vector3<T> uuv = Vector3<T>::cross(qv, uv);
			return point + (uv * w + uuv) * T(2);
		}

		inline constexpr Quaternion& operator*=(T scalar)
		{
			x *= scalar;
			y *= scalar;
			z *= scalar;
			w *= scalar;
			return *this;
		}

		inline constexpr Quaternion& operator/=(T scalar)
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
			*this = Quaternion(
				w * other.x + x * other.w + y * other.z - z * other.y,
				w * other.y + y * other.w + z * other.x - x * other.z,
				w * other.z + z * other.w + x * other.y - y * other.x,
				w * other.w - x * other.x - y * other.y - z * other.z
			);

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

		// Static Member Functions
		static inline T angle(const Quaternion& a, const Quaternion& b)
		{
			T dot_ab = dot(a, b);
			T clamped_dot = clamp(dot_ab, T(-1), T(1));
			return std::acos(clamped_dot) * T(2) * rad_to_deg<T>;
		}

		static inline Quaternion angle_axis(T angle, const Vector3<T>& axis)
		{
			T half_angle = angle * deg_to_rad<T> *T(0.5);
			T sin_half = std::sin(half_angle);
			return Quaternion(
				axis.x * sin_half,
				axis.y * sin_half,
				axis.z * sin_half,
				std::cos(half_angle)
			);
		}

		static inline constexpr Quaternion conjugate(const Quaternion& q)
		{
			return Quaternion(-q.x, -q.y, -q.z, q.w);
		}

		static inline constexpr T dot(const Quaternion& a, const Quaternion& b)
		{
			return a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
		}

		static inline Quaternion euler(T x, T y, T z)
		{
			x *= deg_to_rad<T>;
			y *= deg_to_rad<T>;
			z *= deg_to_rad<T>;

			T half_x = x * T(0.5);
			T half_y = y * T(0.5);
			T half_z = z * T(0.5);

			T sx = std::sin(half_x);
			T sy = std::sin(half_y);
			T sz = std::sin(half_z);

			T cx = std::cos(half_x);
			T cy = std::cos(half_y);
			T cz = std::cos(half_z);

			return Quaternion(
				sx * cy * cz - cx * sy * sz,
				cx * sy * cz + sx * cy * sz,
				cx * cy * sz - sx * sy * cz,
				cx * cy * cz + sx * sy * sz
			);
		}

		static inline Quaternion euler(const Vector3<T>& euler)
		{
			return Quaternion::euler(euler.x, euler.y, euler.z);
		}

		static inline Quaternion from_to_rotation(const Vector3<T>& from_direction, const Vector3<T>& to_direction)
		{
			Vector3<T> f = from_direction.normalized();
			Vector3<T> t = to_direction.normalized();
			T dot_ft = dot(f, t);

			if (dot_ft >= T(1) - epsilon<T>)
			{
				return Quaternion::identity;
			}

			if (dot_ft <= T(-1) + epsilon<T>)
			{
				Vector3<T> axis = Vector3<T>::cross(Vector3<T>::right, f);
				if (axis.length_squared() < epsilon<T>)
				{
					axis = Vector3<T>::cross(Vector3<T>::up, f);
				}

				axis.normalize();
				return angle_axis(pi<T>, axis);
			}

			Vector3<T> c = Vector3<T>::cross(f, t);
			return Quaternion(c.x, c.y, c.z, T(1) + dot_ft).normalized();
		}

		static constexpr Quaternion inverse(const Quaternion& rotation)
		{
			T norm = dot(rotation, rotation);
			if (abs(norm - T(1)) <= epsilon<T>)
			{
				return conjugate(rotation);
			}

			return conjugate(rotation) / norm;
		}

		static inline Quaternion lerp(const Quaternion& a, const Quaternion& b, T t)
		{
			t = clamp01(t);
			return lerp_unclamped(a, b, t);
		}

		static inline Quaternion lerp_unclamped(const Quaternion& a, const Quaternion& b, T t)
		{
			Quaternion result = (dot(a, b) >= T(0)) ? a * (T(1) - t) + b * t : a * (T(1) - t) - b * t;
			return result.normalized();
		}

		static inline Quaternion look_rotation(const Vector3<T>& forward, const Vector3<T>& upwards = Vector3<T>::up)
		{
			Vector3<T> f = forward.normalized();
			Vector3<T> r = Vector3<T>::cross(upwards, f);
			if (r.length_squared() < epsilon<T>)
			{
				if (abs(Vector3<T>::dot(upwards, f) - T(1)) < epsilon<T>)
				{
					return Quaternion::identity;
				}

				return angle_axis(pi<T>, Vector3<T>::cross(f, Vector3<T>::right).normalized());
			}


			r.normalize();
			Vector3<T> u = Vector3<T>::cross(f, r);

			T m00 = r.x, m01 = u.x, m02 = f.x;
			T m10 = r.y, m11 = u.y, m12 = f.y;
			T m20 = r.z, m21 = u.z, m22 = f.z;

			T trace = m00 + m11 + m22;
			if (trace > T(0))
			{
				T s = std::sqrt(trace + T(1)) * T(2);
				return Quaternion(
					(m21 - m12) / s,
					(m02 - m20) / s,
					(m10 - m01) / s,
					T(0.25) * s
				);
			}

			return Quaternion::identity;
		}

		static inline Quaternion rotate_towards(const Quaternion& from, const Quaternion& to, T max_degrees_delta)
		{
			T angle_between = angle(from, to);
			if (angle_between == T(0))
			{
				return to;
			}

			T t = min(T(1), max_degrees_delta / angle_between);
			return slerp(from, to, t);
		}

		static inline Quaternion slerp(const Quaternion& a, const Quaternion& b, T t)
		{
			t = clamp01(t);
			return slerp_unclamped(a, b, t);
		}

		static inline Quaternion slerp_unclamped(const Quaternion& a, const Quaternion& b, T t)
		{
			T dot_ab = dot(a, b);
			Quaternion end = (dot_ab < T(0)) ? -b : b;
			dot_ab = abs(dot_ab);

			if (dot_ab > T(0.9995))
			{
				return (a * (1 - t) + end * t).normalized();
			}

			T theta_0 = std::acos(dot_ab);
			T theta = theta_0 * t;

			Quaternion q = (end - a * dot_ab).normalized();
			return a * std::cos(theta) + q * std::sin(theta);
		}

		// Member Functions
		inline T length() const
		{
			return std::sqrt(x * x + y * y + z * z + w * w);
		}

		inline constexpr T length_squared() const
		{
			return x * x + y * y + z * z + w * w;
		}

		inline constexpr bool is_normalized(T tolerance = epsilon<T>) const
		{
			return abs(T(1) - length_squared()) < tolerance;
		}

		inline void normalize(T tolerance = epsilon<T>)
		{
			*this = normalized(tolerance);
		}

		inline Quaternion normalized(T tolerance = epsilon<T>) const
		{
			T norm = std::sqrt(dot(*this, *this));
			if (norm > tolerance)
			{
				return *this / norm;
			}
			else
			{
				return Quaternion::identity;
			}
		}

		inline constexpr bool equals(const Quaternion& other, T tolerance = loose_epsilon<T>) const
		{
			return abs(x - other.x) <= tolerance && abs(y - other.y) <= tolerance && abs(z - other.z) <= tolerance && abs(w - other.w) <= tolerance;
		}

		inline void set_from_to_rotation(const Vector3<T>& from_direction, const Vector3<T>& to_direction)
		{
			*this = from_to_rotation(from_direction, to_direction);
		}

		inline void set_look_rotation(const Vector3<T>& view, const Vector3<T>& up = Vector3<T>::up)
		{
			*this = look_rotation(view, up);
		}

		inline void to_angle_axis(T& out_angle, Vector3<T>& out_axis) const
		{
			if (w > T(1))
			{
				Quaternion normed = normalized();
				return normed.to_angle_axis(out_angle, out_axis);
			}

			out_angle = T(2) * std::acos(w) * rad_to_deg<T>;

			T s = std::sqrt(T(1) - w * w);
			if (s < epsilon<T>)
			{
				out_axis = Vector3<T>(x, y, z);
			}
			else
			{
				out_axis = Vector3<T>(x / s, y / s, z / s);
			}
		}

		inline constexpr void set(T x, T y, T z, T w)
		{
			this->x = x;
			this->y = y;
			this->z = z;
			this->w = w;
		}

		inline std::string to_string() const
		{
			return std::format("({:.2f}, {:.2f}, {:.2f}, {:.2f})", x, y, z, w);
		}
	};

	template<FloatingPoint T>
	inline const Quaternion<T> Quaternion<T>::identity = Quaternion(T(0), T(0), T(0), T(1));
}

