#include <iostream>

#include "agmath/agmath.hpp"

int main()
{
	for (int i = 0; i < 100; i++)
	{
		std::cout << agm::Rand01() << std::endl;
	}
}

