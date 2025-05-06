#include <iostream>

#include "agmath/agmath.hpp"

int main()
{
	agm::Quaternion<float> rotation = agm::Quaternion<float>::euler(0.f, 90.f, 0.f);
	float angle;
	agm::Vector3<float> axis;
	rotation.to_angle_axis(angle, axis);
	std::cout << angle << " " << axis.to_string() << std::endl;
}

