#include <iostream>

#include "agmath/agmath.h"

using namespace std;
using namespace agm;

int main()
{
	Vector3 v;
	v.x = Sign(-1);
	v.y = Sign(0);
	v.z = Sign(1);
	cout << v.ToString();
}

