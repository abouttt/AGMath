#include <iostream>

#include "agmath/agmath.h"

int main()
{
	agm::Quaternion q = agm::Quaternion::Euler(0.f, 90.f, 0.f);
	float angle;
	agm::Vector3 axis;
	q.ToAngleAxis(angle, axis);
	std::cout << angle << " " << axis.ToString() << std::endl;
}

