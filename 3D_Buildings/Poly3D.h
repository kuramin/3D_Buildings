#pragma once
#include "Contour3D.h"
using namespace std;
class Poly3D
{
public:
	Contour3D outer_contour;
	vector<Contour3D> inner_contours;
	//float foot_height, eaves_height;

	Poly3D() {} // empty constructor
	~Poly3D() {} // destructor
};