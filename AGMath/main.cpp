#include <iostream>

#include "agmath/agmath.hpp"

int main()
{
	agm::Vector2<int> v1(15, 15);
	agm::Vector2<int> v2(15, 15);
	std::cout << v1.to_string();
}

