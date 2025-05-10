#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <format>
#include <string>

#include "utilities.h"
#include "vector3.h"

namespace agm
{
	struct Vector3Int
	{
		static const Vector3Int ZERO;
		static const Vector3Int ONE;
		static const Vector3Int UP;
		static const Vector3Int DOWN;
		static const Vector3Int LEFT;
		static const Vector3Int RIGHT;
		static const Vector3Int FORWARD;
		static const Vector3Int BACK;

		union
		{
			struct
			{
				int32_t x;
				int32_t y;
				int32_t z;
			};

			std::array<int32_t, 3> data;
		};

		constexpr Vector3Int()
			: x(0)
			, y(0)
			, z(0)
		{
		}

		constexpr Vector3Int(int32_t x, int32_t y, int32_t z)
			: x(x)
			, y(y)
			, z(z)
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

		inline constexpr Vector3Int operator-() const
		{
			return Vector3Int(-x, -y, -z);
		}

		inline constexpr Vector3Int operator*(int32_t scalar) const
		{
			return Vector3Int(x * scalar, y * scalar, z * scalar);
		}

		inline constexpr Vector3Int operator/(int32_t scalar) const
		{
			return Vector3Int(x / scalar, y / scalar, z / scalar);
		}

		inline constexpr Vector3Int operator+(const Vector3Int& other) const
		{
			return Vector3Int(x + other.x, y + other.y, z + other.z);
		}

		inline constexpr Vector3Int operator-(const Vector3Int& other) const
		{
			return Vector3Int(x - other.x, y - other.y, z - other.z);
		}

		inline constexpr Vector3Int operator*(const Vector3Int& other) const
		{
			return Vector3Int(x * other.x, y * other.y, z * other.z);
		}

		inline constexpr Vector3Int operator/(const Vector3Int& other) const
		{
			return Vector3Int(x / other.x, y / other.y, z / other.z);
		}

		inline constexpr Vector3Int& operator*=(int32_t scalar)
		{
			x *= scalar;
			y *= scalar;
			z *= scalar;
			return *this;
		}

		inline constexpr Vector3Int& operator/=(int32_t scalar)
		{
			x /= scalar;
			y /= scalar;
			z /= scalar;
			return *this;
		}

		inline constexpr Vector3Int& operator+=(const Vector3Int& other)
		{
			x += other.x;
			y += other.y;
			z += other.z;
			return *this;
		}

		inline constexpr Vector3Int& operator-=(const Vector3Int& other)
		{
			x -= other.x;
			y -= other.y;
			z -= other.z;
			return *this;
		}

		inline constexpr Vector3Int& operator*=(const Vector3Int& other)
		{
			x *= other.x;
			y *= other.y;
			z *= other.z;
			return *this;
		}

		inline constexpr Vector3Int& operator/=(const Vector3Int& other)
		{
			x /= other.x;
			y /= other.y;
			z /= other.z;
			return *this;
		}

		inline constexpr bool operator==(const Vector3Int& other) const
		{
			return x == other.x && y == other.y && z == other.z;
		}

		inline constexpr bool operator!=(const Vector3Int& other) const
		{
			return !(*this == other);
		}

		friend inline constexpr Vector3Int operator*(int32_t scalar, const Vector3Int& v)
		{
			return Vector3Int(v.x * scalar, v.y * scalar, v.z * scalar);
		}

		static inline float Distance(const Vector3Int& a, const Vector3Int& b)
		{
			return (a - b).Length();
		}

		static inline Vector3Int CeilToInt(const Vector3& v)
		{
			return Vector3Int(agm::CeilToInt(v.x), agm::CeilToInt(v.y), agm::CeilToInt(v.z));
		}

		static inline Vector3Int FloorToInt(const Vector3& v)
		{
			return Vector3Int(agm::FloorToInt(v.x), agm::FloorToInt(v.y), agm::FloorToInt(v.z));
		}

		static inline constexpr Vector3Int Max(const Vector3Int& a, const Vector3Int& b)
		{
			return Vector3Int(agm::Max(a.x, b.x), agm::Max(a.y, b.y), agm::Max(a.z, b.z));
		}

		static inline constexpr Vector3Int Min(const Vector3Int& a, const Vector3Int& b)
		{
			return Vector3Int(agm::Min(a.x, b.x), agm::Min(a.y, b.y), agm::Min(a.z, b.z));
		}

		static inline Vector3Int RoundToInt(const Vector3& v)
		{
			return Vector3Int(agm::RoundToInt(v.x), agm::RoundToInt(v.y), agm::RoundToInt(v.z));
		}

		inline constexpr void Clamp(const Vector3Int& min, const Vector3Int& max)
		{
			x = agm::Clamp(x, min.x, max.x);
			y = agm::Clamp(y, min.y, max.y);
			z = agm::Clamp(z, min.z, max.z);
		}

		inline float Length() const
		{
			return std::sqrt(float(x * x + y * y + z * z));
		}

		inline constexpr int32_t LengthSquared() const
		{
			return x * x + y * y + z * z;
		}

		inline constexpr void Set(int32_t x, int32_t y, int32_t z)
		{
			this->x = x;
			this->y = y;
			this->z = z;
		}

		std::string ToString() const
		{
			return std::format("({}, {}, {})", x, y, z);
		}
	};

	inline const Vector3Int Vector3Int::ZERO = Vector3Int(0, 0, 0);
	inline const Vector3Int Vector3Int::ONE = Vector3Int(1, 1, 1);
	inline const Vector3Int Vector3Int::UP = Vector3Int(0, 1, 0);
	inline const Vector3Int Vector3Int::DOWN = Vector3Int(0, -1, 0);
	inline const Vector3Int Vector3Int::LEFT = Vector3Int(-1, 0, 0);
	inline const Vector3Int Vector3Int::RIGHT = Vector3Int(1, 0, 0);
	inline const Vector3Int Vector3Int::FORWARD = Vector3Int(0, 0, 1);
	inline const Vector3Int Vector3Int::BACK = Vector3Int(0, 0, -1);
}

