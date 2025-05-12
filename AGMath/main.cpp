#include <iostream>

#include "agmath/agmath.h"

using namespace std;
using namespace agm;

int main()
{
	auto scale = Vector3(5, 3, 2);
	auto matrix = Matrix4x4::TRS(Vector3::ZERO, Quaternion::IDENTITY, scale);
	cout << matrix.Determinant();
}

