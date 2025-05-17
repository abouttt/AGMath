#include <iostream>

#include "agmath/agmath.h"

int main()
{
	agm::Quaternion myRotation = agm::Quaternion::IDENTITY;
	myRotation.SetEulerAngles(agm::Vector3(150, 35, 45));

	std::cout << myRotation.GetEulerAngles().ToString();
}

