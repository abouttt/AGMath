#include <iostream>

#include "agmath/agmath.hpp"

int main()
{
	agm::Vector3 v(12, 45, 23);
	v.Normalize();
	std::cout << v.length() << std::endl;
}

