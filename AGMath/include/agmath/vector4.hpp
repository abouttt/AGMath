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
	template<Numeric T>
	struct Vector4
	{
		// Static Member Variables
		static const Vector4 zero;
		static const Vector4 one;

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
		constexpr Vector4()
			: x(T(0))
			, y(T(0))
			, z(T(0))
			, w(T(0))
		{
		}

		constexpr Vector4(T x, T y, T z, T w)
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

		inline constexpr Vector4 operator-() const
		{
			return Vector4(-x, -y, -z, -w);
		}

		inline constexpr Vector4 operator+(T scalar) const
		{
			return Vector4(x + scalar, y + scalar, z + scalar, w + scalar);
		}

		inline constexpr Vector4 operator-(T scalar) const
		{
			return Vector4(x - scalar, y - scalar, z - scalar, w - scalar);
		}

		inline constexpr Vector4 operator*(T scalar) const
		{
			return Vector4(x * scalar, y * scalar, z * scalar, w * scalar);
		}

		inline constexpr Vector4 operator/(T scalar) const
		{
			return Vector4(x / scalar, y / scalar, z / scalar, w / scalar);
		}

		inline constexpr Vector4 operator+(const Vector4& other) const
		{
			return Vector4(x + other.x, y + other.y, z + other.z, w + other.w);
		}

		inline constexpr Vector4 operator-(const Vector4& other) const
		{
			return Vector4(x - other.x, y - other.y, z - other.z, w - other.w);
		}

		inline constexpr Vector4 operator*(const Vector4& other) const
		{
			return Vector4(x * other.x, y * other.y, z * other.z, w * other.w);
		}

		inline constexpr Vector4 operator/(const Vector4& other) const
		{
			return Vector4(x / other.x, y / other.y, z / other.z, w / other.w);
		}

		inline constexpr T operator|(const Vector4& other) const
		{
			return x * other.x + y * other.y + z * other.z + w * other.w;
		}

		inline constexpr Vector4& operator+=(T scalar)
		{
			x += scalar;
			y += scalar;
			z += scalar;
			w += scalar;
			return *this;
		}

		inline constexpr Vector4& operator-=(T scalar)
		{
			x -= scalar;
			y -= scalar;
			z -= scalar;
			w -= scalar;
			return *this;
		}

		inline constexpr Vector4& operator*=(T scalar)
		{
			x *= scalar;
			y *= scalar;
			z *= scalar;
			w *= scalar;
			return *this;
		}

		inline constexpr Vector4& operator/=(T scalar)
		{
			x /= scalar;
			y /= scalar;
			z /= scalar;
			w /= scalar;
			return *this;
		}

		inline constexpr Vector4& operator+=(const Vector4& other)
		{
			x += other.x;
			y += other.y;
			z += other.z;
			w += other.w;
			return *this;
		}

		inline constexpr Vector4& operator-=(const Vector4& other)
		{
			x -= other.x;
			y -= other.y;
			z -= other.z;
			w -= other.w;
			return *this;
		}

		inline constexpr Vector4& operator*=(const Vector4& other)
		{
			x *= other.x;
			y *= other.y;
			z *= other.z;
			w *= other.w;
			return *this;
		}

		inline constexpr Vector4& operator/=(const Vector4& other)
		{
			x /= other.x;
			y /= other.y;
			z /= other.z;
			w /= other.w;
			return *this;
		}

		inline constexpr bool operator==(const Vector4& other) const
		{
			return x == other.x && y == other.y && z == other.z && w == other.w;
		}

		inline constexpr bool operator!=(const Vector4& other) const
		{
			return !(*this == other);
		}

		friend inline constexpr Vector4 operator*(T scalar, const Vector4& v)
		{
			return Vector4(v.x * scalar, v.y * scalar, v.z * scalar, v.w * scalar);
		}

		// Static Member Functions
		static inline T distance(const Vector4& a, const Vector4& b)
		{
			return (a - b).length();
		}

		static inline constexpr T dot(const Vector4& a, const Vector4& b)
		{
			return a | b;
		}

		static inline constexpr Vector4 lerp(const Vector4& a, const Vector4& b, T t)
		{
			t = clamp01(t);
			return Vector4(a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t, a.z + (b.z - a.z) * t, a.w + (b.w - a.w) * t);
		}

		static inline constexpr Vector4 lerp_unclamped(const Vector4& a, const Vector4& b, T t)
		{
			return Vector4(a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t, a.z + (b.z - a.z) * t, a.w + (b.w - a.w) * t);
		}

		static inline constexpr Vector4 max(const Vector4& a, const Vector4& b)
		{
			return Vector4(agm::max(a.x, b.x), agm::max(a.y, b.y), agm::max(a.z, b.z), agm::max(a.w, b.w));
		}

		static inline constexpr Vector4 min(const Vector4& a, const Vector4& b)
		{
			return Vector4(agm::min(a.x, b.x), agm::min(a.y, b.y), agm::min(a.z, b.z), agm::min(a.w, b.w));
		}

		static inline Vector4 move_towards(const Vector4& current, const Vector4& target, T max_distance_delta) requires std::is_floating_point_v<T>
		{
			Vector4 delta = target - current;
			T len_sq = delta.length_squared();
			if (agm::is_nearly_zero(len_sq) || (max_distance_delta >= T(0) && len_sq <= max_distance_delta * max_distance_delta))
			{
				return target;
			}

			T len = std::sqrt(len_sq);
			return current + delta / len * max_distance_delta;
		}

		static inline constexpr Vector4 project(const Vector4& v, const Vector4& normal) requires std::is_floating_point_v<T>
		{
			T normal_len_sq = dot(normal, normal);
			if (agm::is_nearly_zero(normal_len_sq))
			{
				return Vector4::zero;
			}

			return normal * dot(v, normal) / normal_len_sq;
		}

		// Member Functions
		inline constexpr bool equals(const Vector4& other, T tolerance = loose_epsilon<T>) const requires std::is_floating_point_v<T>
		{
			return abs(x - other.x) <= tolerance && abs(y - other.y) <= tolerance && abs(z - other.z) <= tolerance && abs(w - other.w) <= tolerance;
		}

		inline constexpr bool is_nearly_zero(T tolerance = loose_epsilon<T>) const requires std::is_floating_point_v<T>
		{
			return abs(x) <= tolerance && abs(y) <= tolerance && abs(z) <= tolerance && abs(w) <= tolerance;
		}

		inline constexpr bool is_zero() const
		{
			return x == T(0) && y == T(0) && z == T(0) && w == T(0);
		}

		inline constexpr bool is_normalized() const
		{
			return (abs(T(1) - length_squared()) < T(0.01));
		}

		inline T length() const
		{
			return T(std::sqrt(x * x + y * y + z * z + w * w));
		}

		inline constexpr T length_squared() const
		{
			return x * x + y * y + z * z + w * w;
		}

		inline void normalize(T tolerance = epsilon<T>) requires std::is_floating_point_v<T>
		{
			T len_sq = length_squared();
			if (len_sq > tolerance * tolerance)
			{
				T inv_len = T(1) / std::sqrt(len_sq);
				*this *= inv_len;
			}
			else
			{
				*this = Vector4::zero;
			}
		}

		inline Vector4 normalized(T tolerance = epsilon<T>) const requires std::is_floating_point_v<T>
		{
			Vector4 result = *this;
			result.normalize(tolerance);
			return result;
		}

		inline constexpr void set(T x, T y, T z, T w)
		{
			this->x = x;
			this->y = y;
			this->z = z;
			this->w = w;
		}

		std::string to_string() const
		{
			if constexpr (std::is_floating_point_v<T>)
			{
				return std::format("({:.2f}, {:.2f}, {:.2f}, {:.2f})", x, y, z, w);
			}
			else
			{
				return std::format("({}, {}, {}, {})", x, y, z, w);
			}
		}
	};

	template<Numeric T>
	inline const Vector4<T> Vector4<T>::zero = Vector4(T(0), T(0), T(0), T(0));

	template<Numeric T>
	inline const Vector4<T> Vector4<T>::one = Vector4(T(1), T(1), T(1), T(1));
}

