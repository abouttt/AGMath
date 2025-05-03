#include <iostream>

#include "agmath/agmath.hpp"

int main()
{
	agm::Vector2<float> v1(15.f, 2.f);
	agm::Vector2<float> v2(45.f, 11.f);
	std::cout << agm::Vector2<float>::angle(v1, v2) << std::endl;
}

