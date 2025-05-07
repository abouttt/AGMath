#include <iostream>

#include "agmath/agmath.hpp"

int main()
{
	for (int i = 0; i < 100; i++)
	{
		std::cout << agm::rand01() << std::endl;
	}
}

