#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <format>
#include <string>
#include <type_traits>

#include "concepts.hpp"
#include "constants.hpp"
#include "utilities.hpp"

namespace agm
{
	template<Arithmetic T>
	struct Vector3
	{
		// Static Member Variables
		static const Vector3 zero;
		static const Vector3 one;
		static const Vector3 up;
		static const Vector3 down;
		static const Vector3 left;
		static const Vector3 right;
		static const Vector3 forward;
		static const Vector3 back;

		// Member Variables
		union
		{
			struct
			{
				T x, y, z;
			};

			std::array<T, 3> data;
		};

		// Constructors
		constexpr Vector3()
			: x(T(0))
			, y(T(0))
			, z(T(0))
		{
		}

		constexpr Vector3(T x, T y, T z)
			: x(x)
			, y(y)
			, z(z)
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

		inline constexpr Vector3 operator-() const
		{
			return Vector3(-x, -y, -z);
		}

		inline constexpr Vector3 operator*(T scalar) const
		{
			return Vector3(x * scalar, y * scalar, z * scalar);
		}

		inline constexpr Vector3 operator/(T scalar) const
		{
			return Vector3(x / scalar, y / scalar, z / scalar);
		}

		inline constexpr Vector3 operator+(const Vector3& other) const
		{
			return Vector3(x + other.x, y + other.y, z + other.z);
		}

		inline constexpr Vector3 operator-(const Vector3& other) const
		{
			return Vector3(x - other.x, y - other.y, z - other.z);
		}

		inline constexpr Vector3 operator*(const Vector3& other) const
		{
			return Vector3(x * other.x, y * other.y, z * other.z);
		}

		inline constexpr Vector3 operator/(const Vector3& other) const
		{
			return Vector3(x / other.x, y / other.y, z / other.z);
		}

		inline constexpr Vector3& operator*=(T scalar)
		{
			x *= scalar;
			y *= scalar;
			z *= scalar;
			return *this;
		}

		inline constexpr Vector3& operator/=(T scalar)
		{
			x /= scalar;
			y /= scalar;
			z /= scalar;
			return *this;
		}

		inline constexpr Vector3& operator+=(const Vector3& other)
		{
			x += other.x;
			y += other.y;
			z += other.z;
			return *this;
		}

		inline constexpr Vector3& operator-=(const Vector3& other)
		{
			x -= other.x;
			y -= other.y;
			z -= other.z;
			return *this;
		}

		inline constexpr Vector3& operator*=(const Vector3& other)
		{
			x *= other.x;
			y *= other.y;
			z *= other.z;
			return *this;
		}

		inline constexpr Vector3& operator/=(const Vector3& other)
		{
			x /= other.x;
			y /= other.y;
			z /= other.z;
			return *this;
		}

		inline constexpr bool operator==(const Vector3& other) const
		{
			return x == other.x && y == other.y && z == other.z;
		}

		inline constexpr bool operator!=(const Vector3& other) const
		{
			return !(*this == other);
		}

		friend inline constexpr Vector3 operator*(T scalar, const Vector3& v)
		{
			return Vector3(v.x * scalar, v.y * scalar, v.z * scalar);
		}

		// Static Member Functions
		static inline T angle(const Vector3& from, const Vector3& to) requires std::is_floating_point_v<T>
		{
			T len = std::sqrt(from.length_squared() * to.length_squared());
			if (agm::is_nearly_zero(len))
			{
				return T(0);
			}

			T cos_theta = clamp(dot(from, to) / len, T(-1), T(1));
			return std::acos(cos_theta) * rad_to_deg<T>;
		}

		static inline Vector3 clamp_length(const Vector3& v, T max_length) requires std::is_floating_point_v<T>
		{
			T len_sq = v.length_squared();
			if (len_sq > max_length * max_length)
			{
				T len = std::sqrt(len_sq);
				Vector3 norm = v / len;
				return norm * max_length;
			}

			return v;
		}

		static inline constexpr Vector3 cross(const Vector3& a, const Vector3& b)
		{
			return Vector3(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x);
		}

		static inline T distance(const Vector3& a, const Vector3& b)
		{
			return (a - b).length();
		}

		static inline constexpr T dot(const Vector3& a, const Vector3& b)
		{
			return a.x * b.x + a.y * b.y + a.z * b.z;
		}

		template<Arithmetic U>
		static inline constexpr Vector3 lerp(const Vector3& a, const Vector3& b, U t)
		{
			t = clamp01(t);
			return Vector3(a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t, a.z + (b.z - a.z) * t);
		}

		template<Arithmetic U>
		static inline constexpr Vector3 lerp_unclamped(const Vector3& a, const Vector3& b, U t)
		{
			return Vector3(a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t, a.z + (b.z - a.z) * t);
		}

		static inline constexpr Vector3 max(const Vector3& a, const Vector3& b)
		{
			return Vector3(agm::max(a.x, b.x), agm::max(a.y, b.y), agm::max(a.z, b.z));
		}

		static inline constexpr Vector3 min(const Vector3& a, const Vector3& b)
		{
			return Vector3(agm::min(a.x, b.x), agm::min(a.y, b.y), agm::min(a.z, b.z));
		}

		static inline Vector3 move_towards(const Vector3& current, const Vector3& target, T max_distance_delta) requires std::is_floating_point_v<T>
		{
			Vector3 delta = target - current;
			T len_sq = delta.length_squared();
			if (agm::is_nearly_zero(len_sq) || (max_distance_delta >= T(0) && len_sq <= max_distance_delta * max_distance_delta))
			{
				return target;
			}

			T len = std::sqrt(len_sq);
			return current + delta / len * max_distance_delta;
		}

		static inline void ortho_normalize(Vector3& inout_normal, Vector3& inout_tangent) requires std::is_floating_point_v<T>
		{
			inout_normal.normalize();
			inout_tangent = project_on_plane(inout_tangent, inout_normal);
			inout_tangent.normalize();
		}

		static inline void ortho_normalize(Vector3& inout_normal, Vector3& inout_tangent, Vector3& inout_binormal) requires std::is_floating_point_v<T>
		{
			ortho_normalize(inout_normal, inout_tangent);
			inout_binormal = cross(inout_normal, inout_tangent);
			inout_binormal.normalize();
		}

		static inline constexpr Vector3 project(const Vector3& v, const Vector3& normal) requires std::is_floating_point_v<T>
		{
			T normal_len_sq = dot(normal, normal);
			if (agm::is_nearly_zero(normal_len_sq))
			{
				return Vector3::zero;
			}

			return normal * dot(v, normal) / normal_len_sq;
		}

		static inline constexpr Vector3 project_on_plane(const Vector3& v, const Vector3& plane_normal) requires std::is_floating_point_v<T>
		{
			T normal_len_sq = dot(plane_normal, plane_normal);
			if (agm::is_nearly_zero(normal_len_sq))
			{
				return v;
			}

			return v - plane_normal * dot(v, plane_normal) / normal_len_sq;
		}

		static inline constexpr Vector3 reflect(const Vector3& direction, const Vector3& normal)
		{
			return (T(-2) * dot(direction, normal)) * normal + direction;
		}

		static inline T signed_angle(const Vector3& from, const Vector3& to, const Vector3& axis)
		{
			return angle(from, to) * sign(dot(axis, cross(from, to)));
		}

		static inline Vector3 slerp(const Vector3& a, const Vector3& b, T t) requires std::is_floating_point_v<T>
		{
			t = clamp01(t);
			return slerp_unclamped(a, b, t);
		}

		static inline Vector3 slerp_unclamped(const Vector3& a, const Vector3& b, T t) requires std::is_floating_point_v<T>
		{
			Vector3 from = a.normalized();
			Vector3 to = b.normalized();

			T dot_ab = clamp(dot(from, to), T(-1), T(1));
			T theta = std::acos(dot_ab);

			if (theta < epsilon<T>)
			{
				return from;
			}

			T sin_theta = std::sin(theta);
			T scale_a = std::sin((T(1) - t) * theta) / sin_theta;
			T scale_b = std::sin(t * theta) / sin_theta;

			return from * scale_a + to * scale_b;
		}

		// Member Functions
		inline constexpr bool equals(const Vector3& other, T tolerance = loose_epsilon<T>) const requires std::is_floating_point_v<T>
		{
			return abs(x - other.x) <= tolerance && abs(y - other.y) <= tolerance && abs(z - other.z) <= tolerance;
		}

		inline constexpr bool equals(const Vector3& other) const requires std::is_integral_v<T>
		{
			return x == other.x && y == other.y && z == other.z;
		}

		inline constexpr bool is_nearly_zero(T tolerance = loose_epsilon<T>) const requires std::is_floating_point_v<T>
		{
			return abs(x) <= tolerance && abs(y) <= tolerance && abs(z) <= tolerance;
		}

		inline constexpr bool is_zero() const
		{
			return x == T(0) && y == T(0) && z == T(0);
		}

		inline constexpr bool is_normalized(T tolerance = epsilon<T>) const requires std::is_floating_point_v<T>
		{
			return abs(T(1) - length_squared()) < tolerance;
		}

		inline T length() const
		{
			return T(std::sqrt(x * x + y * y + z * z));
		}

		inline constexpr T length_squared() const
		{
			return x * x + y * y + z * z;
		}

		inline void normalize(T tolerance = epsilon<T>) requires std::is_floating_point_v<T>
		{
			*this = normalized(tolerance);
		}

		inline Vector3 normalized(T tolerance = epsilon<T>) const requires std::is_floating_point_v<T>
		{
			T len_sq = length_squared();
			if (len_sq > tolerance)
			{
				T inv_len = T(1) / std::sqrt(len_sq);
				return *this * inv_len;
			}
			else
			{
				return Vector3::zero;
			}
		}

		inline constexpr void set(T x, T y, T z)
		{
			this->x = x;
			this->y = y;
			this->z = z;
		}

		std::string to_string() const
		{
			if constexpr (std::is_floating_point_v<T>)
			{
				return std::format("({:.2f}, {:.2f}, {:.2f})", x, y, z);
			}
			else
			{
				return std::format("({}, {}, {})", x, y, z);
			}
		}
	};

	template<Arithmetic T>
	inline const Vector3<T> Vector3<T>::zero = Vector3(T(0), T(0), T(0));

	template<Arithmetic T>
	inline const Vector3<T> Vector3<T>::one = Vector3(T(1), T(1), T(1));

	template<Arithmetic T>
	inline const Vector3<T> Vector3<T>::up = Vector3(T(0), T(1), T(0));

	template<Arithmetic T>
	inline const Vector3<T> Vector3<T>::down = Vector3(T(0), T(-1), T(0));

	template<Arithmetic T>
	inline const Vector3<T> Vector3<T>::left = Vector3(T(-1), T(0), T(0));

	template<Arithmetic T>
	inline const Vector3<T> Vector3<T>::right = Vector3(T(1), T(0), T(0));

	template<Arithmetic T>
	inline const Vector3<T> Vector3<T>::forward = Vector3(T(0), T(0), T(1));

	template<Arithmetic T>
	inline const Vector3<T> Vector3<T>::back = Vector3(T(0), T(0), T(-1));
}

