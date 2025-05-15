#pragma once

#include <cmath>
#include <cstdint>

namespace agm
{
	inline constexpr float EPSILON = 1e-6f;

	inline constexpr float PI = 3.141592653589793f;
	inline constexpr float TWO_PI = 6.283185307179586f;
	inline constexpr float HALF_PI = 1.5707963267948966f;
	inline constexpr float INV_PI = 0.3183098861837907f;

	inline constexpr float DEG2RAD = 0.017453292519943295f;
	inline constexpr float RAD2DEG = 57.29577951308232f;

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
		return x < min ? min : x > max ? max : x;
	}

	inline constexpr float Clamp01(float value)
	{
		return value < 0.f ? 0.f : value > 1.f ? 1.f : value;
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
	inline constexpr bool InRange(T x, T minInclusive, T maxExclusive)
	{
		return x >= minInclusive && x < maxExclusive;
	}

	inline constexpr float Sign(float value)
	{
		return value > 0.f ? 1.f : value < 0.f ? -1.f : 0.f;
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

		return Clamp(value - std::floor(value / length) * length, 0.f, length);
	}

	inline float PingPong(float t, float length)
	{
		t = Repeat(t, length * 2.f);
		return length - Abs(t - length);
	}

	inline float DeltaAngle(float current, float target)
	{
		float delta = Repeat(target - current, 360.f);
		if (delta > 180.f)
		{
			delta -= 360.f;
		}

		return delta;
	}

	inline float WrapAngle(float angle)
	{
		return Repeat(angle + 180.f, 360.f) - 180.f;
	}

	inline float AngleBetween(float a, float b)
	{
		return Abs(WrapAngle(b - a));
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
		return a + DeltaAngle(a, b) * Clamp01(t);
	}

	inline constexpr float InverseLerp(float a, float b, float value)
	{
		if (IsNearlyEqual(a, b))
		{
			return 0.f;
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
		if (-maxDelta < delta && delta < maxDelta)
		{
			return target;
		}

		return MoveTowards(current, current + delta, maxDelta);
	}

	inline constexpr float SmoothStep(float from, float to, float t)
	{
		t = Clamp01(t);
		t = t * t * (3.f - 2.f * t);
		return to * t + from * (1.f - t);
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

		int32_t nextPower = NextPowerOfTwo(value);
		int32_t prevPower = nextPower >> 1;
		return (value - prevPower < nextPower - value) ? prevPower : nextPower;
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

