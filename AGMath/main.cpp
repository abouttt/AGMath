#include <iostream>

#include "agmath/agmath.h"

using namespace std;
using namespace agm;

int main()
{
	for (int i = 0; i < 100; i++)
	{
		cout << Rand01() << endl;
	}
}

