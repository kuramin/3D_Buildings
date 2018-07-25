#pragma once
#include "Poly3D.h"
using namespace std;
class Building3D
{
public:
	Poly3D* polygon;
	float foot_height, eaves_height;

	Building3D() {} // empty constructor
	~Building3D() {} // destructor
};