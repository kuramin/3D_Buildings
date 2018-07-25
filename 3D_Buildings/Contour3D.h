#pragma once
#include "Point3.h"
using namespace std;
class Contour3D
{
public:
	vector<Point3> points;

	Contour3D() {} // empty constructor
	~Contour3D() {} // destructor
};