#include <iostream>

#include "agmath/agmath.hpp"

int main()
{
	agm::Vector3<float> v1(15.f, 2.f, 34.f);
	agm::Vector3<float> v2(45.f, 11.f, 5.f);
	std::cout << agm::Vector3<float>::angle(v1, v2) << std::endl;
}

