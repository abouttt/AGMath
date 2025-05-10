#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <format>
#include <string>

#include "utilities.h"
#include "vector2.h"

namespace agm
{
	struct Vector2Int
	{
		static const Vector2Int ZERO;
		static const Vector2Int ONE;
		static const Vector2Int UP;
		static const Vector2Int DOWN;
		static const Vector2Int LEFT;
		static const Vector2Int RIGHT;

		union
		{
			struct
			{
				int32_t x, y;
			};

			std::array<int32_t, 2> data;
		};

		constexpr Vector2Int()
			: x(0)
			, y(0)
		{
		}

		constexpr Vector2Int(int32_t x, int32_t y)
			: x(x)
			, y(y)
		{
		}

		inline constexpr int32_t operator[](size_t index) const
		{
			return data[index];
		}

		inline constexpr int32_t& operator[](size_t index)
		{
			return data[index];
		}

		inline constexpr Vector2Int operator-() const
		{
			return Vector2Int(-x, -y);
		}

		inline constexpr Vector2Int operator*(int32_t scalar) const
		{
			return Vector2Int(x * scalar, y * scalar);
		}

		inline constexpr Vector2Int operator/(int32_t scalar) const
		{
			return Vector2Int(x / scalar, y / scalar);
		}

		inline constexpr Vector2Int operator+(const Vector2Int& other) const
		{
			return Vector2Int(x + other.x, y + other.y);
		}

		inline constexpr Vector2Int operator-(const Vector2Int& other) const
		{
			return Vector2Int(x - other.x, y - other.y);
		}

		inline constexpr Vector2Int operator*(const Vector2Int& other) const
		{
			return Vector2Int(x * other.x, y * other.y);
		}

		inline constexpr Vector2Int operator/(const Vector2Int& other) const
		{
			return Vector2Int(x / other.x, y / other.y);
		}

		inline constexpr Vector2Int& operator*=(int32_t scalar)
		{
			x *= scalar;
			y *= scalar;
			return *this;
		}

		inline constexpr Vector2Int& operator/=(int32_t scalar)
		{
			x /= scalar;
			y /= scalar;
			return *this;
		}

		inline constexpr Vector2Int& operator+=(const Vector2Int& other)
		{
			x += other.x;
			y += other.y;
			return *this;
		}

		inline constexpr Vector2Int& operator-=(const Vector2Int& other)
		{
			x -= other.x;
			y -= other.y;
			return *this;
		}

		inline constexpr Vector2Int& operator*=(const Vector2Int& other)
		{
			x *= other.x;
			y *= other.y;
			return *this;
		}

		inline constexpr Vector2Int& operator/=(const Vector2Int& other)
		{
			x /= other.x;
			y /= other.y;
			return *this;
		}

		inline constexpr bool operator==(const Vector2Int& other) const
		{
			return x == other.x && y == other.y;
		}

		inline constexpr bool operator!=(const Vector2Int& other) const
		{
			return !(*this == other);
		}

		friend inline constexpr Vector2Int operator*(int32_t scalar, const Vector2Int& v)
		{
			return Vector2Int(v.x * scalar, v.y * scalar);
		}

		static inline float Distance(const Vector2Int& a, const Vector2Int& b)
		{
			return (a - b).Length();
		}

		static inline Vector2Int CeilToInt(const Vector2& v)
		{
			return Vector2Int(agm::CeilToInt(v.x), agm::CeilToInt(v.y));
		}

		static inline Vector2Int FloorToInt(const Vector2& v)
		{
			return Vector2Int(agm::FloorToInt(v.x), agm::FloorToInt(v.y));
		}

		static inline constexpr Vector2Int Max(const Vector2Int& a, const Vector2Int& b)
		{
			return Vector2Int(agm::Max(a.x, b.x), agm::Max(a.y, b.y));
		}

		static inline constexpr Vector2Int Min(const Vector2Int& a, const Vector2Int& b)
		{
			return Vector2Int(agm::Min(a.x, b.x), agm::Min(a.y, b.y));
		}

		static inline Vector2Int RoundToInt(const Vector2& v)
		{
			return Vector2Int(agm::RoundToInt(v.x), agm::RoundToInt(v.y));
		}

		inline constexpr void Clamp(const Vector2Int& min, const Vector2Int& max)
		{
			x = agm::Clamp(x, min.x, max.x);
			y = agm::Clamp(y, min.y, max.y);
		}

		inline float Length() const
		{
			return std::sqrt(float(x * x + y * y));
		}

		inline constexpr int32_t LengthSquared() const
		{
			return x * x + y * y;
		}

		inline constexpr void Set(int32_t x, int32_t y)
		{
			this->x = x;
			this->y = y;
		}

		std::string ToString() const
		{
			return std::format("({}, {})", x, y);
		}
	};

	inline const Vector2Int Vector2Int::ZERO = Vector2Int(0, 0);
	inline const Vector2Int Vector2Int::ONE = Vector2Int(1, 1);
	inline const Vector2Int Vector2Int::UP = Vector2Int(0, 1);
	inline const Vector2Int Vector2Int::DOWN = Vector2Int(0, -1);
	inline const Vector2Int Vector2Int::LEFT = Vector2Int(-1, 0);
	inline const Vector2Int Vector2Int::RIGHT = Vector2Int(1, 0);
}

