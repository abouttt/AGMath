#include <iostream>

#include "agmath/agmath.h"

using namespace std;
using namespace agm;

int main()
{
	float t = 0.f;
	while (true)
	{
		cout << SmoothStep(0.f, 100.f, t) << endl;
		t += 0.01f;
		if (t >= 1.f)
		{
			break;
		}
	}
}

