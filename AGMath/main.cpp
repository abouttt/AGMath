#include <iostream>

#include "agmath/agmath.hpp"

int main()
{
	agm::Vector4<float> v1(15.f, 2.f, 34.f, 89.f);
	agm::Vector4<float> v2(45.f, 11.f, 5.f, 452.f);
	v1 = agm::Vector4<float>::zero;
	std::cout << v1.is_nearly_zero() << std::endl;
}

