#pragma once

#include <cmath>

#include "utilities.h" 

namespace agm
{
	// Linear
	inline constexpr float EaseLinear(float t)
	{
		return Clamp01(t);
	}

	//
	// Ease-In
	//

	// Quadratic Ease-In
	inline constexpr float EaseInQuad(float t)
	{
		t = Clamp01(t);
		return t * t;
	}

	// Cubic Ease-In
	inline constexpr float EaseInCubic(float t)
	{
		t = Clamp01(t);
		return t * t * t;
	}

	// Quartic Ease-In
	inline constexpr float EaseInQuart(float t)
	{
		t = Clamp01(t);
		return t * t * t * t;
	}

	// Quintic Ease-In
	inline constexpr float EaseInQuint(float t)
	{
		t = Clamp01(t);
		return t * t * t * t * t;
	}

	// Sine Ease-In
	inline float EaseInSine(float t)
	{
		t = Clamp01(t);
		return 1.f - std::cos((t * PI) * 0.5f);
	}

	// Exponential Ease-In
	inline float EaseInExpo(float t)
	{
		t = Clamp01(t);
		if (IsNearlyZero(t))
		{
			return 0.f;
		}

		return std::pow(2.f, 10.f * (t - 1.f));
	}

	// Circular Ease-In
	inline float EaseInCirc(float t)
	{
		t = Clamp01(t);
		return 1.f - std::sqrt(1.f - t * t);
	}

	// Elastic Ease-In
	inline float EaseInElastic(float t)
	{
		t = Clamp01(t);
		if (IsNearlyZero(t))
		{
			return 0.f;
		}
		if (IsNearlyEqual(t, 1.f))
		{
			return 1.f;
		}

		constexpr float p = 0.3f;
		constexpr float s = p / 4.f;
		float t_adj = t - 1.f;
		return -std::pow(2.f, 10.f * t_adj) * std::sin((t_adj - s) * TWO_PI / p);
	}

	//
	// Ease-Out
	//

	// Quadratic Ease-Out
	inline constexpr float EaseOutQuad(float t)
	{
		t = Clamp01(t);
		return t * (2.f - t);
	}

	// Cubic Ease-Out
	inline constexpr float EaseOutCubic(float t)
	{
		t = Clamp01(t);
		const float u = 1.f - t;
		return 1.f - u * u * u;
	}

	// Quartic Ease-Out
	inline constexpr float EaseOutQuart(float t)
	{
		t = Clamp01(t);
		const float u = 1.f - t;
		return 1.f - u * u * u * u;
	}

	// Quintic Ease-Out
	inline constexpr float EaseOutQuint(float t)
	{
		t = Clamp01(t);
		const float u = 1.f - t;
		return 1.f - u * u * u * u * u;
	}

	// Sine Ease-Out
	inline float EaseOutSine(float t)
	{
		t = Clamp01(t);
		return std::sin((t * PI) * 0.5f);
	}

	// Exponential Ease-Out
	inline float EaseOutExpo(float t)
	{
		t = Clamp01(t);
		if (IsNearlyEqual(t, 1.f))
		{
			return 1.f;
		}

		return 1.f - std::pow(2.f, -10.f * t);
	}

	// Circular Ease-Out
	inline float EaseOutCirc(float t)
	{
		t = Clamp01(t);
		const float u = t - 1.f;
		return std::sqrt(1.f - u * u);
	}

	// Elastic Ease-Out
	inline float EaseOutElastic(float t)
	{
		t = Clamp01(t);
		if (IsNearlyZero(t))
		{
			return 0.f;
		}
		if (IsNearlyEqual(t, 1.f))
		{
			return 1.f;
		}

		constexpr float p = 0.3f;
		constexpr float s = p / 4.f;
		return std::pow(2.f, -10.f * t) * std::sin((t - s) * TWO_PI / p) + 1.f;
	}

	//
	// Ease-In-Out
	//

	// Quadratic Ease-In-Out
	inline constexpr float EaseInOutQuad(float t)
	{
		t = Clamp01(t);
		if (t < 0.5f)
		{
			return 2.f * t * t;
		}
		else
		{
			const float u = -2.f * t + 2.f;
			return 1.f - (u * u) * 0.5f;
		}
	}

	// Cubic Ease-In-Out
	inline constexpr float EaseInOutCubic(float t)
	{
		t = Clamp01(t);
		if (t < 0.5f)
		{
			return 4.f * t * t * t;
		}
		else
		{
			const float u = -2.f * t + 2.f;
			return 1.f - (u * u * u) * 0.5f;
		}
	}

	// Quartic Ease-In-Out
	inline constexpr float EaseInOutQuart(float t)
	{
		t = Clamp01(t);
		if (t < 0.5f)
		{
			return 8.f * t * t * t * t;
		}
		else
		{
			const float u = -2.f * t + 2.f;
			return 1.f - (u * u * u * u) * 0.5f;
		}
	}

	// Quintic Ease-In-Out
	inline constexpr float EaseInOutQuint(float t)
	{
		t = Clamp01(t);
		if (t < 0.5f)
		{
			return 16.f * t * t * t * t * t;
		}
		else
		{
			const float u = -2.f * t + 2.f;
			return 1.f - (u * u * u * u * u) * 0.5f;
		}
	}

	// Sine Ease-In-Out
	inline float EaseInOutSine(float t)
	{
		t = Clamp01(t);
		return -(std::cos(PI * t) - 1.f) * 0.5f;
	}

	// Exponential Ease-In-Out
	inline float EaseInOutExpo(float t)
	{
		t = Clamp01(t);
		if (IsNearlyZero(t))
		{
			return 0.f;
		}
		if (IsNearlyEqual(t, 1.f))
		{
			return 1.f;
		}

		if (t < 0.5f)
		{
			return std::pow(2.f, 20.f * t - 10.f) * 0.5f;
		}
		else
		{
			return (2.f - std::pow(2.f, -20.f * t + 10.f)) * 0.5f;
		}
	}

	// Circular Ease-In-Out
	inline float EaseInOutCirc(float t)
	{
		t = Clamp01(t);
		if (t < 0.5f)
		{
			const float u = 2.f * t;
			return (1.f - std::sqrt(1.f - u * u)) * 0.5f;
		}
		else
		{
			const float u = -2.f * t + 2.f;
			return (std::sqrt(1.f - u * u) + 1.f) * 0.5f;
		}
	}

	// Elastic Ease-In-Out
	inline float EaseInOutElastic(float t)
	{
		t = Clamp01(t);
		if (IsNearlyZero(t))
		{
			return 0.f;
		}
		if (IsNearlyEqual(t, 1.f))
		{
			return 1.f;
		}

		constexpr float p = 0.3f * 1.5f;
		const float scaled_time_phase = (20.f * t) - 11.125f;

		if (t < 0.5f)
		{
			return -0.5f * (std::pow(2.f, 20.f * t - 10.f) * std::sin(scaled_time_phase * TWO_PI / p));
		}
		else
		{
			return std::pow(2.f, -20.f * t + 10.f) * std::sin(scaled_time_phase * TWO_PI / p) * 0.5f + 1.f;
		}
	}

	//
	// Special Effects
	//

	// Bounce Ease-Out
	inline constexpr float EaseOutBounce(float t)
	{
		t = Clamp01(t);

		constexpr float n1 = 7.5625f;
		constexpr float d1 = 2.75f;

		if (t < 1.f / d1)
		{
			return n1 * t * t;
		}
		else if (t < 2.f / d1)
		{
			float t_adj = t - (1.5f / d1);
			return n1 * t_adj * t_adj + 0.75f;
		}
		else if (t < 2.5f / d1)
		{
			float t_adj = t - (2.25f / d1);
			return n1 * t_adj * t_adj + 0.9375f;
		}
		else
		{
			float t_adj = t - (2.625f / d1);
			return n1 * t_adj * t_adj + 0.984375f;
		}
	}

	// Bounce Ease-In
	inline constexpr float EaseInBounce(float t)
	{
		t = Clamp01(t);
		return 1.f - EaseOutBounce(1.f - t);
	}

	// Bounce Ease-In-Out
	inline constexpr float EaseInOutBounce(float t)
	{
		t = Clamp01(t);
		if (t < 0.5f)
		{
			return EaseInBounce(2.f * t) * 0.5f;
		}
		else
		{
			return EaseOutBounce(2.f * t - 1.f) * 0.5f + 0.5f;
		}
	}

	// Back Ease-In
	inline constexpr float EaseInBack(float t)
	{
		t = Clamp01(t);
		constexpr float c1 = 1.70158f;
		constexpr float c3 = c1 + 1.f;
		return c3 * t * t * t - c1 * t * t;
	}

	// Back Ease-Out
	inline constexpr float EaseOutBack(float t)
	{
		t = Clamp01(t);
		constexpr float c1 = 1.70158f;
		constexpr float c3 = c1 + 1.f;
		const float u = t - 1.f;
		return 1.f + c3 * u * u * u + c1 * u * u;
	}

	// Back Ease-In-Out
	inline constexpr float EaseInOutBack(float t)
	{
		t = Clamp01(t);

		constexpr float c1 = 1.70158f;
		constexpr float c2 = c1 * 1.525f;

		if (t < 0.5f)
		{
			const float u = 2.f * t;
			return (u * u * ((c2 + 1.f) * u - c2)) * 0.5f;
		}
		else
		{
			const float u = 2.f * t - 2.f;
			return (u * u * ((c2 + 1.f) * u + c2) + 2.f) * 0.5f;
		}
	}
}

