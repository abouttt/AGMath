#pragma once

#include "concepts.hpp"

namespace agm
{
	template<Floating T>
	inline constexpr T epsilon = static_cast<T>(1e-8);

	template<Floating T>
	inline constexpr T loose_epsilon = static_cast<T>(1e-4);

	template<Floating T>
	inline constexpr T golden_ratio = static_cast<T>(1.618033988749895);

	template<Floating T>
	inline constexpr T pi = static_cast<T>(3.141592653589793);

	template<Floating T>
	inline constexpr T two_pi = static_cast<T>(6.283185307179586);

	template<Floating T>
	inline constexpr T half_pi = static_cast<T>(1.5707963267948966);

	template<Floating T>
	inline constexpr T inv_pi = static_cast<T>(0.3183098861837907);

	template<Floating T>
	inline constexpr T deg_to_rad = static_cast<T>(0.017453292519943295);

	template<Floating T>
	inline constexpr T rad_to_deg = static_cast<T>(57.29577951308232);

	template<Floating T>
	inline constexpr T sqrt2 = static_cast<T>(1.4142135623730951);

	template<Floating T>
	inline constexpr T sqrt3 = static_cast<T>(1.7320508075688773);

	template<Floating T>
	inline constexpr T half_sqrt2 = static_cast<T>(0.7071067811865476);

	template<Floating T>
	inline constexpr T half_sqrt3 = static_cast<T>(0.8660254037844386);

	template<Floating T>
	inline constexpr T inv_sqrt2 = static_cast<T>(0.7071067811865476);

	template<Floating T>
	inline constexpr T inv_sqrt3 = static_cast<T>(0.5773502691896258);

	template<Floating T>
	inline constexpr T e = static_cast<T>(2.718281828459045);

	template<Floating T>
	inline constexpr T log2 = static_cast<T>(0.6931471805599453);

	template<Floating T>
	inline constexpr T log10 = static_cast<T>(2.302585092994046);

	template<Floating T>
	inline constexpr T log2e = static_cast<T>(1.4426950408889634);

	template<Floating T>
	inline constexpr T log10e = static_cast<T>(0.4342944819032518);
}

