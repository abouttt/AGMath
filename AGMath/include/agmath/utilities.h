#pragma once

#include <cmath>
#include <cstdint>
#include <numbers>

namespace agm
{
	inline constexpr float EPSILON = 1e-6f;
	inline constexpr float EPSILON_SQ_LENGTH = 1e-9f;
	inline constexpr float EPSILON_DOT_ONE = 1e-5f;
	inline constexpr float MATRIX_EPSILON = 1e-5f;

	inline constexpr float VECTOR_NORMALIZED_THRESHOLD = 0.01f;
	inline constexpr float QUATERNION_NORMALIZED_THRESHOLD = 0.01f;

	inline constexpr float PI = std::numbers::pi_v<float>;
	inline constexpr float TWO_PI = 2.f * PI;
	inline constexpr float HALF_PI = 0.5f * PI;
	inline constexpr float INV_PI = 1.f / PI;

	inline constexpr float DEG2RAD = PI / 180.f;
	inline constexpr float RAD2DEG = 180.f / PI;

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
		return value >= 0.f ? 1.f : -1.f;
	}

	template<typename T>
	inline constexpr T Square(T x)
	{
		return x * x;
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

		float result = value - std::floor(value / length) * length;
		return IsNearlyEqual(result, length) ? 0.f : result;
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

	inline constexpr float Remap(float value, float oldMin, float oldMax, float newMin, float newMax)
	{
		float t = InverseLerp(oldMin, oldMax, value);
		return Lerp(newMin, newMax, t);
	}

	inline constexpr float MoveTowards(float current, float target, float maxDelta)
	{
		if (maxDelta < 0.f)
		{
			maxDelta = 0.f;
		}

		float delta = target - current;
		if (Abs(delta) <= maxDelta)
		{
			return target;
		}

		return current + Sign(delta) * maxDelta;
	}

	inline float MoveTowardsAngle(float current, float target, float maxDelta)
	{
		if (maxDelta < 0.f)
		{
			maxDelta = 0.f;
		}

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

	inline constexpr float SmoothInverseLerp(float from, float to, float value)
	{
		float t = InverseLerp(from, to, value);
		return t * t * (3.f - 2.f * t);
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
		return (value & 1) != 0;
	}

	inline float Sqrt(float value)
	{
		return std::sqrt(value);
	}

	inline float InvSqrt(float value)
	{
		if (IsNearlyZero(value, EPSILON_SQ_LENGTH))
		{
			return 0.f;
		}

		return 1.f / std::sqrt(value);
	}

	inline float Sin(float radians)
	{
		return std::sin(radians);
	}

	inline float Cos(float radians)
	{
		return std::cos(radians);
	}

	inline float Tan(float radians)
	{
		return std::tan(radians);
	}

	inline float Asin(float value)
	{
		return std::asin(Clamp(value, -1.f, 1.f));
	}

	inline float Acos(float value)
	{
		return std::acos(Clamp(value, -1.f, 1.f));
	}

	inline float Atan(float value)
	{
		return std::atan(value);
	}

	inline float Atan2(float y, float x)
	{
		return std::atan2(y, x);
	}

	inline float Exp(float power)
	{
		return std::exp(power);
	}

	inline float Log(float value, float base)
	{
		return std::log(value) / std::log(base);
	}

	inline float Log2(float value)
	{
		return std::log2(value);
	}

	inline float Log10(float value)
	{
		return std::log10(value);
	}

	inline int32_t FloorToInt(float value)
	{
		return static_cast<int32_t>(std::floor(value));
	}

	inline int32_t CeilToInt(float value)
	{
		return static_cast<int32_t>(std::ceil(value));
	}

	inline int32_t RoundToInt(float value)
	{
		return static_cast<int32_t>(std::round(value));
	}
}

