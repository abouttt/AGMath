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
	struct Vector2
	{
		// Static Member Variables
		static const Vector2 zero;
		static const Vector2 one;
		static const Vector2 up;
		static const Vector2 down;
		static const Vector2 left;
		static const Vector2 right;

		// Member Variables
		union
		{
			struct
			{
				T x, y;
			};

			std::array<T, 2> data;
		};

		// Constructors
		constexpr Vector2()
			: x(T(0))
			, y(T(0))
		{
		}

		constexpr Vector2(T x, T y)
			: x(x)
			, y(y)
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

		inline constexpr Vector2 operator-() const
		{
			return Vector2(-x, -y);
		}

		inline constexpr Vector2 operator+(T scalar) const
		{
			return Vector2(x + scalar, y + scalar);
		}

		inline constexpr Vector2 operator-(T scalar) const
		{
			return Vector2(x - scalar, y - scalar);
		}

		inline constexpr Vector2 operator*(T scalar) const
		{
			return Vector2(x * scalar, y * scalar);
		}

		inline constexpr Vector2 operator/(T scalar) const
		{
			return Vector2(x / scalar, y / scalar);
		}

		inline constexpr Vector2 operator+(const Vector2& other) const
		{
			return Vector2(x + other.x, y + other.y);
		}

		inline constexpr Vector2 operator-(const Vector2& other) const
		{
			return Vector2(x - other.x, y - other.y);
		}

		inline constexpr Vector2 operator*(const Vector2& other) const
		{
			return Vector2(x * other.x, y * other.y);
		}

		inline constexpr Vector2 operator/(const Vector2& other) const
		{
			return Vector2(x / other.x, y / other.y);
		}

		inline constexpr T operator|(const Vector2& other) const
		{
			return x * other.x + y * other.y;
		}

		inline constexpr T operator^(const Vector2& other) const
		{
			return x * other.y - y * other.y;
		}

		inline constexpr Vector2& operator+=(T scalar)
		{
			x += scalar;
			y += scalar;
			return *this;
		}

		inline constexpr Vector2& operator-=(T scalar)
		{
			x -= scalar;
			y -= scalar;
			return *this;
		}

		inline constexpr Vector2& operator*=(T scalar)
		{
			x *= scalar;
			y *= scalar;
			return *this;
		}

		inline constexpr Vector2& operator/=(T scalar)
		{
			x /= scalar;
			y /= scalar;
			return *this;
		}

		inline constexpr Vector2& operator+=(const Vector2& other)
		{
			x += other.x;
			y += other.y;
			return *this;
		}

		inline constexpr Vector2& operator-=(const Vector2& other)
		{
			x -= other.x;
			y -= other.y;
			return *this;
		}

		inline constexpr Vector2& operator*=(const Vector2& other)
		{
			x *= other.x;
			y *= other.y;
			return *this;
		}

		inline constexpr Vector2& operator/=(const Vector2& other)
		{
			x /= other.x;
			y /= other.y;
			return *this;
		}

		inline constexpr bool operator==(const Vector2& other) const
		{
			return x == other.x && y == other.y;
		}

		inline constexpr bool operator!=(const Vector2& other) const
		{
			return !(*this == other);
		}

		friend inline constexpr Vector2 operator*(T scalar, const Vector2& v)
		{
			return Vector2(v.x * scalar, v.y * scalar);
		}

		// Static Member Functions
		static inline T angle(const Vector2& from, const Vector2& to) requires std::is_floating_point_v<T>
		{
			T len = std::sqrt(from.length_squared() * to.length_squared());
			if (agm::is_nearly_zero(len))
			{
				return T(0);
			}

			T cos_theta = clamp(dot(from, to) / len, T(-1), T(1));
			return std::acos(cos_theta) * rad_to_deg<T>;
		}

		static inline Vector2 clamp_length(const Vector2& v, T max_length) requires std::is_floating_point_v<T>
		{
			T len_sq = v.length_squared();
			if (len_sq > max_length * max_length)
			{
				T len = std::sqrt(len_sq);
				Vector2 norm = v / len;
				return norm * max_length;
			}

			return v;
		}

		static inline constexpr T cross(const Vector2& a, const Vector2& b)
		{
			return a ^ b;
		}

		static inline T distance(const Vector2& a, const Vector2& b)
		{
			return (a - b).length();
		}

		static inline constexpr T dot(const Vector2& a, const Vector2& b)
		{
			return a | b;
		}

		static inline constexpr Vector2 lerp(const Vector2& a, const Vector2& b, T t)
		{
			t = clamp01(t);
			return Vector2(a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t);
		}

		static inline constexpr Vector2 lerp_unclamped(const Vector2& a, const Vector2& b, T t)
		{
			return Vector2(a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t);
		}

		static inline constexpr Vector2 max(const Vector2& a, const Vector2& b)
		{
			return Vector2(agm::max(a.x, b.x), agm::max(a.y, b.y));
		}

		static inline constexpr Vector2 min(const Vector2& a, const Vector2& b)
		{
			return Vector2(agm::min(a.x, b.x), agm::min(a.y, b.y));
		}

		static inline Vector2 move_towards(const Vector2& current, const Vector2& target, T max_distance_delta) requires std::is_floating_point_v<T>
		{
			Vector2 delta = target - current;
			T len_sq = delta.length_squared();
			if (agm::is_nearly_zero(len_sq) || (max_distance_delta >= T(0) && len_sq <= max_distance_delta * max_distance_delta))
			{
				return target;
			}

			T len = std::sqrt(len_sq);
			return current + delta / len * max_distance_delta;
		}

		static inline constexpr Vector2 perpendicular(const Vector2& v)
		{
			return Vector2(-v.y, v.x);
		}

		static inline constexpr Vector2 reflect(const Vector2& direction, const Vector2& normal)
		{
			return (T(-2) * dot(direction, normal)) * normal + direction;
		}

		static inline T signed_angle(const Vector2& from, const Vector2& to)
		{
			return angle(from, to) * sign(from.x * to.y - from.y * to.x);
		}

		// Member Functions
		inline constexpr bool equals(const Vector2& other, T tolerance = loose_epsilon<T>) const requires std::is_floating_point_v<T>
		{
			return abs(x - other.x) <= tolerance && abs(y - other.y) <= tolerance;
		}

		inline constexpr bool is_nearly_zero(T tolerance = loose_epsilon<T>) const requires std::is_floating_point_v<T>
		{
			return abs(x) <= tolerance && abs(y) <= tolerance;
		}

		inline constexpr bool is_zero() const
		{
			return x == T(0) && y == T(0);
		}

		inline constexpr bool is_normalized() const
		{
			return (abs(T(1) - length_squared()) < T(0.01));
		}

		inline T length() const
		{
			return T(std::sqrt(x * x + y * y));
		}

		inline constexpr T length_squared() const
		{
			return x * x + y * y;
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
				*this = Vector2::zero;
			}
		}

		inline Vector2 normalized(T tolerance = epsilon<T>) const requires std::is_floating_point_v<T>
		{
			Vector2 result = *this;
			result.normalize(tolerance);
			return result;
		}

		inline constexpr void set(T x, T y)
		{
			this->x = x;
			this->y = y;
		}

		std::string to_string() const
		{
			if constexpr (std::is_floating_point_v<T>)
			{
				return std::format("({:.2f}, {:.2f})", x, y);
			}
			else
			{
				return std::format("({}, {})", x, y);
			}
		}
	};

	template<Numeric T>
	inline const Vector2<T> Vector2<T>::zero = Vector2(T(0), T(0));

	template<Numeric T>
	inline const Vector2<T> Vector2<T>::one = Vector2(T(1), T(1));

	template<Numeric T>
	inline const Vector2<T> Vector2<T>::up = Vector2(T(0), T(1));

	template<Numeric T>
	inline const Vector2<T> Vector2<T>::down = Vector2(T(0), T(-1));

	template<Numeric T>
	inline const Vector2<T> Vector2<T>::left = Vector2(T(-1), T(0));

	template<Numeric T>
	inline const Vector2<T> Vector2<T>::right = Vector2(T(1), T(0));
}

