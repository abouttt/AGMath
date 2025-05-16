#pragma once

#include <cmath>

#include "utilities.h"

namespace agm
{
	// Linear
	inline constexpr float EaseLinear(float t)
	{
		return t;
	}

	//
	// Ease-In
	//

	// Quadratic Ease-In
	inline constexpr float EaseInQuad(float t)
	{
		return t * t;
	}

	// Cubic Ease-In
	inline constexpr float EaseInCubic(float t)
	{
		return t * t * t;
	}

	// Quartic Ease-In
	inline constexpr float EaseInQuart(float t)
	{
		return t * t * t * t;
	}

	// Quintic Ease-In
	inline constexpr float EaseInQuint(float t)
	{
		return t * t * t * t * t;
	}

	// Sine Ease-In
	inline float EaseInSine(float t)
	{
		return 1.f - std::cos((t * PI) * 0.5f);
	}

	// Exponential Ease-In
	inline float EaseInExpo(float t)
	{
		if (t == 0.f)
		{
			return 0.f;
		}

		return std::pow(2.f, 10.f * (t - 1.f));
	}

	// Circular Ease-In
	inline float EaseInCirc(float t)
	{
		return 1.f - std::sqrt(1.f - t * t);
	}

	// Elastic Ease-In
	inline float EaseInElastic(float t)
	{
		if (t == 0.f)
		{
			return 0.f;
		}

		if (t == 1.f)
		{
			return 1.f;
		}

		constexpr float p = 0.3f;
		constexpr float s = p / 4.f;
		return -std::pow(2.f, 10.f * (t -= 1.f)) * std::sin((t - s) * (TWO_PI) / p);
	}

	//
	// Ease-Out
	// 

	// Quadratic Ease-Out
	inline constexpr float EaseOutQuad(float t)
	{
		return 1.f - (1.f - t) * (1.f - t);
	}

	// Cubic Ease-Out
	inline float EaseOutCubic(float t)
	{
		return 1.f - std::pow(1.f - t, 3.f);
	}

	// Quartic Ease-Out
	inline float EaseOutQuart(float t)
	{
		return 1.f - std::pow(1.f - t, 4.f);
	}

	// Quintic Ease-Out
	inline float EaseOutQuint(float t)
	{
		return 1.f - std::pow(1.f - t, 5.f);
	}

	// Sine Ease-Out
	inline float EaseOutSine(float t)
	{
		return std::sin((t * PI) * 0.5f);
	}

	// Exponential Ease-Out
	inline float EaseOutExpo(float t)
	{
		if (t == 1.f)
		{
			return 1.f;
		}

		return 1.f - std::pow(2.f, -10.f * t);
	}

	// Circular Ease-Out
	inline float EaseOutCirc(float t)
	{
		return std::sqrt(1.f - std::pow(t - 1.f, 2.f));
	}

	// Elastic Ease-Out
	inline float EaseOutElastic(float t)
	{
		if (t == 0.f)
		{
			return 0.f;
		}

		if (t == 1.f)
		{
			return 1.f;
		}

		constexpr float p = 0.3f;
		constexpr float s = p / 4.f;
		return std::pow(2.f, -10.f * t) * std::sin((t - s) * (TWO_PI) / p) + 1.f;
	}

	//
	// Ease-In-Out
	// 

	// Quadratic Ease-In-Out
	inline float EaseInOutQuad(float t)
	{
		if (t < 0.5f)
		{
			return 2.f * t * t;
		}
		else
		{
			return 1.f - std::pow(-2.f * t + 2.f, 2.f) * 0.5f;
		}
	}

	// Cubic Ease-In-Out
	inline float EaseInOutCubic(float t)
	{
		if (t < 0.5f)
		{
			return 4.f * t * t * t;
		}
		else
		{
			return 1.f - std::pow(-2.f * t + 2.f, 3.f) * 0.5f;
		}
	}

	// Quartic Ease-In-Out
	inline float EaseInOutQuart(float t)
	{
		if (t < 0.5f)
		{
			return 8.f * t * t * t * t;
		}
		else
		{
			return 1.f - std::pow(-2.f * t + 2.f, 4.f) * 0.5f;
		}
	}

	// Quintic Ease-In-Out
	inline float EaseInOutQuint(float t)
	{
		if (t < 0.5f)
		{
			return 16.f * t * t * t * t * t;
		}
		else
		{
			return 1.f - std::pow(-2.f * t + 2.f, 5.f) * 0.5f;
		}
	}

	// Sine Ease-In-Out
	inline float EaseInOutSine(float t)
	{
		return -(std::cos(PI * t) - 1.f) * 0.5f;
	}

	// Exponential Ease-In-Out
	inline float EaseInOutExpo(float t)
	{
		if (t == 0.f)
		{
			return 0.f;
		}

		if (t == 1.f)
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
		if (t < 0.5f)
		{
			return (1.f - std::sqrt(1.f - std::pow(2.f * t, 2.f))) * 0.5f;
		}
		else
		{
			return (std::sqrt(1.f - std::pow(-2.f * t + 2.f, 2.f)) + 1.f) * 0.5f;
		}
	}

	// Elastic Ease-In-Out
	inline float EaseInOutElastic(float t)
	{
		if (t == 0.f)
		{
			return 0.f;
		}

		if (t == 1.f)
		{
			return 1.f;
		}

		constexpr float p = 0.3f * 1.5f;
		constexpr float s = p / 4.f;

		if (t < 0.5f)
		{
			return -0.5f * (std::pow(2.f, 20.f * t - 10.f) * std::sin((20.f * t - 11.125f) * TWO_PI / p));
		}
		else
		{
			return std::pow(2.f, -20.f * t + 10.f) * std::sin((20.f * t - 11.125f) * TWO_PI / p) * 0.5f + 1.f;
		}
	}

	//
	// Special Effects
	//

	// Bounce Ease-Out
	inline constexpr float EaseOutBounce(float t)
	{
		constexpr float n1 = 7.5625f;
		constexpr float d1 = 2.75f;

		if (t < 1.f / d1)
		{
			return n1 * t * t;
		}
		else if (t < 2.f / d1)
		{
			return n1 * (t -= 1.5f / d1) * t + 0.75f;
		}
		else if (t < 2.5f / d1)
		{
			return n1 * (t -= 2.25f / d1) * t + 0.9375f;
		}
		else
		{
			return n1 * (t -= 2.625f / d1) * t + 0.984375f;
		}
	}

	// Bounce Ease-In
	inline constexpr float EaseInBounce(float t)
	{
		return 1.f - EaseOutBounce(1.f - t);
	}

	// Bounce Ease-In-Out
	inline constexpr float EaseInOutBounce(float t)
	{
		if (t < 0.5f)
		{
			return (1.f - EaseOutBounce(1.f - 2.f * t)) * 0.5f;
		}
		else
		{
			return (1.f + EaseOutBounce(2.f * t - 1.f)) * 0.5f;
		}
	}

	// Back Ease-In
	inline constexpr float EaseInBack(float t)
	{
		constexpr float c1 = 1.70158f;
		constexpr float c3 = c1 + 1.f;
		return c3 * t * t * t - c1 * t * t;
	}

	// Back Ease-Out
	inline float EaseOutBack(float t)
	{
		constexpr float c1 = 1.70158f;
		constexpr float c3 = c1 + 1.f;
		return 1.f + c3 * std::pow(t - 1.f, 3.f) + c1 * std::pow(t - 1.f, 2.f);
	}

	// Back Ease-In-Out
	inline float EaseInOutBack(float t)
	{
		constexpr float c1 = 1.70158f;
		constexpr float c2 = c1 * 1.525f;

		if (t < 0.5f)
		{
			return (std::pow(2.f * t, 2.f) * ((c2 + 1.f) * 2.f * t - c2)) * 0.5f;
		}
		else
		{
			return (std::pow(2.f * t - 2.f, 2.f) * ((c2 + 1.f) * (t * 2.f - 2.f) + c2) + 2.f) * 0.5f;
		}
	}
}

