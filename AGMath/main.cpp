#include <iostream>

#include "agmath/agmath.h"

using namespace std;
using namespace agm;

int main()
{
	auto matrix = Matrix4x4(Vector4(1, 2, 3, 4), Vector4(5, 6, 7, 8), Vector4(9, 10, 11, 12), Vector4(13, 14, 15, 16));
	std::cout << matrix.Transpose().ToString();
}

