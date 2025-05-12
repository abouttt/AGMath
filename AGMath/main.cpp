#include <iostream>

#include "agmath/agmath.h"

using namespace std;
using namespace agm;

int main()
{
    Quaternion myRotation = Quaternion::IDENTITY;
    myRotation.SetEulerAngles(Vector3(150, 35, 45));
    cout << myRotation.GetEulerAngles().ToString();
}

