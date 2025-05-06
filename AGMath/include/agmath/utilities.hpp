#pragma once

#include <algorithm>
#include <cmath>

#include "concepts.hpp"
#include "constants.hpp"

namespace agm
{
	template<Numeric T>
	inline constexpr T clamp01(T a)
	{
		return std::clamp(a, T(0), T(1));
	}

	template<typename T>
	inline constexpr bool in_range(T x, T min_inclusive, T max_exclusive)
	{
		return x >= min_inclusive && x < max_exclusive;
	}

	template<Numeric T>
	inline constexpr T sign(T value)
	{
		return (value > T(0)) - (value < T(0));
	}

	template<FloatingPoint T>
	inline constexpr bool is_nearly_equal(T a, T b, T tolerance = epsilon<T>)
	{
		return std::abs(a - b) <= tolerance;
	}

	template<FloatingPoint T>
	inline constexpr bool is_nearly_zero(T value, T tolerance = epsilon<T>)
	{
		return std::abs(value) <= tolerance;
	}

	template<Integral T>
	inline constexpr T next_power_of_two(T value)
	{
		if (value <= 1)
		{
			return 1;
		}

		value--;
		value |= value >> 1;
		value |= value >> 2;
		value |= value >> 4;
		value |= value >> 8;
		value |= value >> 16;
		value++;

		return value;
	}

	template<Integral T>
	inline constexpr T closest_power_of_two(T value)
	{
		if (value <= 0)
		{
			return 0;
		}

		T next_power = next_power_of_two(value);
		T prev_power = next_power >> 1;
		return (value - prev_power < next_power - value) ? prev_power : next_power;
	}

	template<Integral T>
	inline constexpr bool is_power_of_two(T value)
	{
		return (value > 0) && (value & (value - 1)) == 0;
	}

	template<Integral T>
	inline constexpr T wrap(T value, T min, T max)
	{
		T range = max - min;
		if (range == 0)
		{
			return min;
		}

		T offset = (value - min) % range;
		return offset >= 0 ? min + offset : min + offset + range;
	}

	template<FloatingPoint T>
	inline constexpr T wrap(T value, T min, T max)
	{
		T range = max - min;
		return value - range * std::floor((value - min) / range);
	}

	template<Integral T>
	inline constexpr T mod(T value, T divisor)
	{
		if (divisor == 0)
		{
			return 0;
		}

		return (value % divisor + divisor) % divisor;
	}

	template<FloatingPoint T>
	inline T repeat(T value, T length)
	{
		if (length == T(0))
		{
			return T(0);
		}

		return std::clamp(value - std::floor(value / length) * length, T(0), length);
	}

	template<FloatingPoint T>
	inline T ping_pong(T t, T length)
	{
		t = repeat(t, length * T(2));
		return length - std::abs(t - length);
	}

	template<FloatingPoint T>
	inline T delta_angle(T current, T target)
	{
		T delta = repeat(target - current, T(360));
		if (delta > T(180))
		{
			delta -= T(360);
		}

		return delta;
	}

	template<FloatingPoint T>
	inline T normalize_angle(T angle)
	{
		return repeat(angle, T(360));
	}

	template<Numeric T, Numeric U>
	inline constexpr T lerp(T a, T b, U t)
	{
		return T(a + (b - a) * clamp01(t));
	}

	template<Numeric T, Numeric U>
	inline constexpr T lerp_unclamped(T a, T b, U t)
	{
		return T(a + (b - a) * t);
	}

	template<FloatingPoint T>
	inline T lerp_angle(T a, T b, T t)
	{
		return a + delta_angle(a, b) * clamp01(t);
	}

	template<FloatingPoint T>
	inline constexpr T inverse_lerp(T a, T b, T value)
	{
		if (is_nearly_equal(a, b))
		{
			return T(0);
		}

		return clamp01((value - a) / (b - a));
	}

	template<FloatingPoint T>
	inline constexpr T remap(T value, T old_min, T old_max, T new_min, T new_max)
	{
		T t = inverse_lerp(old_min, old_max, value);
		return lerp(new_min, new_max, t);
	}

	template<FloatingPoint T>
	inline constexpr T move_towards(T current, T target, T max_delta)
	{
		if (is_nearly_equal(current, target, max_delta))
		{
			return target;
		}

		return current + sign(target - current) * max_delta;
	}

	template<FloatingPoint T>
	inline T move_towards_angle(T current, T target, T max_delta)
	{
		T delta = delta_angle(current, target);
		if (T(0) - max_delta < delta && delta < max_delta)
		{
			return target;
		}

		target = current + delta;
		return move_towards(current, target, max_delta);
	}

	template<Numeric T, Numeric U>
	inline constexpr T smoothstep(T from, T to, U t)
	{
		t = clamp01(t);
		t = t * t * (U(3) - U(2) * t);
		return T(to * t + from * (U(1) - t));
	}
}

