#pragma once

#include <cmath>
#include <cstdint>
#include <numbers>

namespace agm
{
	inline constexpr float EPSILON = 1e-6f;

	inline constexpr float PI = std::numbers::pi_v<float>;
	inline constexpr float TWO_PI = 2.f * PI;
	inline constexpr float HALF_PI = 0.5f * PI;
	inline constexpr float INV_PI = 1.f / PI;

	inline constexpr float DEG2RAD = PI / 180.f;
	inline constexpr float RAD2DEG = 180.f / PI;

	inline constexpr float THRESH_VECTOR_NORMALIZED = 0.01f;
	inline constexpr float THRESH_QUAT_NORMALIZED = 0.01f;

	template<typename T>
	inline constexpr T Abs(T value)
	{
		return value < T(0) ? -value : value;
	}

	template<typename T>
	inline constexpr T Clamp(T x, T min, T max)
	{
		return x < min ? min : (x > max ? max : x);
	}

	inline constexpr float Clamp01(float value)
	{
		return value < 0.f ? 0.f : (value > 1.f ? 1.f : value);
	}

	inline constexpr double Clamp01(double value)
	{
		return value < 0.0 ? 0.0 : (value > 1.0 ? 1.0 : value);
	}

	template<typename T>
	inline constexpr T Max(T a, T b)
	{
		return a > b ? a : b;
	}

	template<typename T>
	inline constexpr T Max3(T a, T b, T c)
	{
		return Max(Max(a, b), c);
	}

	template<typename T>
	inline constexpr T Min(T a, T b)
	{
		return a < b ? a : b;
	}

	template<typename T>
	inline constexpr T Min3(T a, T b, T c)
	{
		return Min(Min(a, b), c);
	}

	template<typename T>
	inline constexpr bool IsInRange(T x, T min, T max)
	{
		return x >= min && x < max;
	}

	inline constexpr float Sign(float value)
	{
		return value > 0.f ? 1.f : (value < 0.f ? -1.f : 0.f);
	}

	template<typename T>
	inline constexpr T Square(T value)
	{
		return value * value;
	}

	inline constexpr bool IsNearlyEqual(float a, float b, float tolerance = EPSILON)
	{
		return Abs(a - b) <= tolerance;
	}

	inline constexpr bool IsNearlyZero(float value, float tolerance = EPSILON)
	{
		return Abs(value) <= tolerance;
	}

	inline float Repeat(float value, float length)
	{
		if (IsNearlyZero(length))
		{
			return 0.f;
		}

		return value - std::floor(value / length) * length;
	}

	inline float PingPong(float t, float length)
	{
		t = Repeat(t, length * 2.f);
		return length - Abs(t - length);
	}

	inline float WrapAngle(float angle)
	{
		return Repeat(angle + 180.f, 360.f) - 180.f;
	}

	inline float DeltaAngle(float current, float target)
	{
		return WrapAngle(target - current);
	}

	inline constexpr float Lerp(float a, float b, float t)
	{
		return a + (b - a) * Clamp01(t);
	}

	inline constexpr float LerpUnclamped(float a, float b, float t)
	{
		return a + (b - a) * t;
	}

	inline float LerpAngle(float a, float b, float t)
	{
		float delta = DeltaAngle(a, b);
		return WrapAngle(a + delta * Clamp01(t));
	}

	inline constexpr float InverseLerp(float a, float b, float value)
	{
		if (IsNearlyEqual(a, b))
		{
			return value <= a ? 0.f : 1.f;
		}

		return Clamp01((value - a) / (b - a));
	}

	inline float Remap(float value, float oldMin, float oldMax, float newMin, float newMax)
	{
		float t = InverseLerp(oldMin, oldMax, value);
		return Lerp(newMin, newMax, t);
	}

	inline constexpr float MoveTowards(float current, float target, float maxDelta)
	{
		float delta = target - current;
		if (Abs(delta) <= maxDelta)
		{
			return target;
		}

		return current + Sign(delta) * maxDelta;
	}

	inline float MoveTowardsAngle(float current, float target, float maxDelta)
	{
		float delta = DeltaAngle(current, target);
		if (Abs(delta) <= maxDelta)
		{
			return WrapAngle(target);
		}

		return WrapAngle(current + Sign(delta) * maxDelta);
	}

	inline constexpr float SmoothStep(float from, float to, float t)
	{
		t = Clamp01(t);
		t = t * t * (3.f - 2.f * t);
		return from + (to - from) * t;
	}

	inline constexpr float SmoothStep01(float from, float to, float value)
	{
		float t;

		if (IsNearlyEqual(from, to))
		{
			t = value <= from ? 0.f : 1.f;
		}
		else
		{
			t = (value - from) / (to - from);
		}

		t = Clamp01(t);
		t = t * t * (3.f - 2.f * t);
		return t;
	}

	inline constexpr float SmootherStep(float from, float to, float t)
	{
		t = Clamp01(t);
		t = t * t * t * (t * (t * 6.f - 15.f) + 10.f);
		return from + (to - from) * t;
	}

	inline constexpr float SmootherStep01(float from, float to, float value)
	{
		float t;

		if (IsNearlyEqual(from, to))
		{
			t = value <= from ? 0.f : 1.f;
		}
		else
		{
			t = (value - from) / (to - from);
		}

		t = Clamp01(t);
		t = t * t * t * (t * (t * 6.f - 15.f) + 10.f);
		return t;
	}

	inline constexpr int32_t NextPowerOfTwo(int32_t value)
	{
		if (value <= 0)
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

	inline constexpr int32_t ClosestPowerOfTwo(int32_t value)
	{
		if (value <= 0)
		{
			return 1;
		}

		int32_t next = NextPowerOfTwo(value);
		int32_t prev = next >> 1;
		if (prev == 0)
		{
			return next;
		}

		return (value - prev < next - value) ? prev : next;
	}

	inline constexpr bool IsPowerOfTwo(int32_t value)
	{
		return (value > 0) && (value & (value - 1)) == 0;
	}

	inline constexpr bool IsEven(int32_t value)
	{
		return (value & 1) == 0;
	}

	inline constexpr bool IsOdd(int32_t value)
	{
		return (value & 1) == 1;
	}
}

