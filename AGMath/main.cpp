#include <iostream>

#include "agmath/agmath.h"

int main()
{
	agm::Quaternion myRotation = agm::Quaternion::IDENTITY;
	myRotation.FromEulerAngles(agm::Vector3(150, 35, 45));
	std::cout << myRotation.ToEulerAngles().ToString() << std::endl;
	std::cout << agm::WrapAngle(215) << std::endl;
	std::cout << agm::WrapAngle(225) << std::endl;
}

