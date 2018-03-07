#pragma once
#include "Contour3D.h"
using namespace std;
class Poly3D
{
public:
	Contour3D outer_contour;
	vector<Contour3D> inner_contours;

	Poly3D() {} // empty constructor
	~Poly3D() {} // destructor
};