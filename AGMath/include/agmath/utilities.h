#pragma once

#include <cmath>
#include <concepts>

namespace agm
{
	inline constexpr float EPSILON = 1e-8f;
	inline constexpr float LOOSE_EPSILON = 1e-4f;
	inline constexpr float PI = 3.141592653589793f;
	inline constexpr float TWO_PI = 6.283185307179586f;
	inline constexpr float HALF_PI = 1.5707963267948966f;
	inline constexpr float INV_PI = 0.3183098861837907f;
	inline constexpr float DEG2RAD = 0.017453292519943295f;
	inline constexpr float RAD2DEG = 57.29577951308232f;

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
		return value >= 0.f ? 1.f : -1.f;
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
		if (length == 0.f)
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

	template<typename T, typename U>
	inline constexpr T Lerp(T a, T b, U t)
	{
		return (T)(a + (b - a) * Clamp01(t));
	}

	template<typename T, typename U>
	inline constexpr T LerpUnclamped(T a, T b, U t)
	{
		return (T)(a + (b - a) * t);
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
		if (IsNearlyEqual(target, current, maxDelta))
		{
			return target;
		}

		return current + Sign(target - current) * maxDelta;
	}

	inline float MoveTowardsAngle(float current, float target, float maxDelta)
	{
		float delta = DeltaAngle(current, target);
		if (0.f - maxDelta < delta && delta < maxDelta)
		{
			return target;
		}

		target = current + delta;
		return MoveTowards(current, target, maxDelta);
	}

	inline constexpr float SmoothStep(float from, float to, float t)
	{
		t = Clamp01(t);
		t = -2.f * t * t * t + 3.f * t * t;
		return to * t + from * (1.f - t);
	}

	template<std::integral T>
	inline constexpr T NextPowerOfTwo(T value)
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

	template<std::integral T>
	inline constexpr T ClosestPowerOfTwo(T value)
	{
		if (value <= 0)
		{
			return 0;
		}

		T nextPower = NextPowerOfTwo(value);
		T prevPower = nextPower >> 1;
		return (value - prevPower < nextPower - value) ? prevPower : nextPower;
	}

	template<std::integral T>
	inline constexpr bool IsPowerOfTwo(T value)
	{
		return (value > 0) && (value & (value - 1)) == 0;
	}

	inline float Pow(float base, float exponent)
	{
		return std::pow(base, exponent);
	}

	inline float Exp(float value)
	{
		return std::exp(value);
	}

	inline float Sqrt(float value)
	{
		return std::sqrt(value);
	}

	inline float InvSqrt(float value)
	{
		return 1.f / std::sqrt(value);
	}

	inline float Sin(float rad)
	{
		return std::sin(rad);
	}

	inline float Cos(float rad)
	{
		return std::cos(rad);
	}

	inline float Tan(float rad)
	{
		return std::tan(rad);
	}

	inline float Asin(float value)
	{
		return std::asin(value);
	}

	inline float Acos(float value)
	{
		return std::acos(value);
	}

	inline float Atan(float value)
	{
		return std::atan(value);
	}

	inline float Atan2(float y, float x)
	{
		return std::atan2(y, x);
	}

	inline float Log(float value)
	{
		return std::log(value);
	}

	inline float Log(float value, float base)
	{
		return std::log(value) / std::log(base);
	}

	inline float Log10(float value)
	{
		return std::log10(value);
	}

	inline float Ceil(float value)
	{
		return std::ceil(value);
	}

	inline int32_t CeilToInt(float value)
	{
		return (int32_t)std::ceil(value);
	}

	inline float Floor(float value)
	{
		return std::floor(value);
	}

	inline int32_t FloorToInt(float value)
	{
		return (int32_t)std::floor(value);
	}

	inline float Round(float value)
	{
		return std::round(value);
	}

	inline int32_t RoundToInt(float value)
	{
		return (int32_t)std::round(value);
	}

	inline float GammaToLinearSpace(float value)
	{
		if (value <= 0.04045f)
		{
			return value / 12.92f;
		}

		return Pow((value + 0.055f) / 1.055f, 2.4f);
	}

	inline float LinearToGammaSpace(float value)
	{
		if (value <= 0.0031308f)
		{
			return value * 12.92f;
		}

		return 1.055f * Pow(value, 1.0f / 2.4f) - 0.055f;
	}
}

