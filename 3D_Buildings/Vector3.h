#pragma once

#define _USE_MATH_DEFINES

#include <math.h>
#include <string>

class Vector3
{

friend class Point3;

public:
	double x, y, z;

// ---------------------------------------------------------------------------------------------------- //

public:
	// Constructs a vector without explicit initialization.
	inline Vector3() {}

	// Constructs a vector with coordinates (x, y).
	inline Vector3(const double x, const double y, const double z) 
		: x(x), y(y), z(z) {}

	// Constructs a copy of vector v.
	inline Vector3(const Vector3 &v) 
		: x(v.x), y(v.y), z(v.z) {}

// ---------------------------------------------------------------------------------------------------- //

	// Returns true if this vector equals vector v.
	inline bool operator==(const Vector3 &v) const
	{
		const double EPSILON = 0.000000000000001;
		return abs(x - v.x) < EPSILON && abs(y - v.y) < EPSILON && abs(z - v.z) < EPSILON;
	}

	// Returns false if this vector equals vector v.
	inline bool operator!=(const Vector3 &v) const
	{
		const double EPSILON = 0.000000000000001;
		return abs(x - v.x) > EPSILON || abs(y - v.y) > EPSILON || abs(z - v.z) > EPSILON;
	}

	//! Returns true if this vector comes before vector v in a non-ambiguous sorting order.
	inline bool operator<(const Vector3 &v) const
	{
		if (x < v.x)
			return true;
		else if (x > v.x)
			return false;
		else if (y < v.y)
			return true;
		else if (y > v.y)
			return false;
		else
			return z < v.z;
	}

	// Returns the result vector of the addition of this vector and vector v.
	inline Vector3 operator+(const Vector3 &v) const
	{
		return Vector3(x+v.x, y+v.y, z+v.z);
	}

	// Adds vector v to this vector.
	inline Vector3 & operator+=(const Vector3 &v)
	{
		x += v.x;
		y += v.y;
		z += v.z;
		return *this;
	}

	// Returns the result vector of the subtraction of vector v from this vector.
	inline Vector3 operator-(const Vector3 &v) const
	{
		return Vector3(x-v.x, y-v.y, z-v.z);
	}

	// Subtracts vector v from this vector.
	inline Vector3 & operator-=(const Vector3 &v)
	{
		x -= v.x;
		y -= v.y;
		z -= v.z;
		return *this;
	}

	// Returns the result vector of the multiplication of this vector with scalar s.
	inline Vector3 operator*(const double &s) const
	{
		return Vector3(x*s, y*s, z*s);
	}

	// Multiplies this vector with scalar s.
	inline Vector3 & operator*=(const double s)
	{
		x *= s;
		y *= s;
		z *= s;
		return *this;
	}

	// Returns the result vector of the division of this vector by scalar s.
	inline Vector3 operator/(const double s) const
	{
		double f = 1.0 / s;
		return Vector3(x*f, y*f, z*f);
	}

	// Divides this vector by scalar s.
	inline Vector3 & operator/=(const double s)
	{
		double f = 1.0 / s;
		x *= f;
		y *= f;
		z *= f;
		return *this;
	}

	// Returns the negative vector of this vector.
	inline Vector3 operator-() const
	{
		return Vector3(-x, -y, -z);
	}

// ---------------------------------------------------------------------------------------------------- //

	// Returns the length of this vector.
	inline double Length() const
	{
		return sqrt(x*x + y*y + z*z);
	}

		// Returns the 2D length of this vector.
	inline double Length2D() const
	{
		return sqrt(x*x + y*y);
	}

};

// ---------------------------------------------------------------------------------------------------- //

// Writes vector v to output stream os.
inline std::ostream & operator<<(std::ostream &os, const Vector3 &v)
{
	os << v.x << " " << v.y << " " << v.z;
	return os;
}

// Returns the dot product of vector a and vector b.
inline double DotProduct2D(const Vector3 &a, const Vector3 &b)
{
	return a.x*b.x + a.y*b.y;
}

// Returns the angle between vector a and vector b in degree.
inline double Angle2DDegree(const Vector3 &a, const Vector3 &b)
{
	double x = DotProduct2D(a, b) / (a.Length2D() * b.Length2D());

	if (x < -1.0)
		x = -1.0;
	else if (x > 1.0)
		x = 1.0;

	return 180.0 / M_PI * acos(x);
}

// Returns the angle between vector a and b in degree in counterclockwise orientation.
inline double Angle2DDegree360(const Vector3 &a, const Vector3 &b)
{
	// angle between 0 and 180
	double angle = Angle2DDegree(a, b);

	// angle between 0 and 360
	if (DotProduct2D(Vector3(b.y, -b.x, 0.0), a) < 0.0)
		angle = 360.0 - angle;

	return angle;
}