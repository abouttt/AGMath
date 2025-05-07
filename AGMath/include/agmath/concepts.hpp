#pragma once

#include <type_traits>

namespace agm
{
	template<typename T>
	concept Integral = std::is_integral_v<T>;

	template<typename T>
	concept FloatingPoint = std::is_floating_point_v<T>;

	template<typename T>
	concept Arithmetic = std::is_arithmetic_v<T>;
}

