#include <iostream>

#include "agmath/agmath.h"

using namespace std;
using namespace agm;

int main()
{
	Quaternion rotation = Quaternion::Euler(0, 90, 0);
	float angle;
	Vector3 axis;
	rotation.ToAngleAxis(angle, axis);
	cout << angle << " " << axis.ToString() << endl;
}

