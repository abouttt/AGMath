#include <iostream>

#include "agmath/agmath.h"

int main()
{
	agm::Matrix4x4 m = agm::Matrix4x4::IDENTITY;
	int index;
	std::cin >> index;
	m(index, index) = 1.0f;
}

