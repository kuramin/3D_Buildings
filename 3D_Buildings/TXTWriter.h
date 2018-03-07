#pragma once

#include "Point3.h"

#include <fstream>
#include <iomanip>
#include <vector>

using namespace std;

class TXTWriter
{

public:
	static void WritePointXYZ(std::string path, const Point3 point, ios_base::openmode mode)
	{
		std::ofstream file;
		file.open(path, mode);

		file << std::fixed << std::showpoint;
		file << std::setprecision(6);

		file << point << std::endl;

		file.close();
	}

	static void WritePointXYZ(std::string path, const std::vector<Point3> &points, ios_base::openmode mode)
	{
		if (points.size() == 0)
			return;

		std::ofstream file;
		file.open(path, mode); // std::ios::out);

		file << std::fixed << std::showpoint;
		file << std::setprecision(6);

		for (const Point3 &p : points)
		{
			file << p << std::endl;
		}

		file.close();
	}

	static void WritePointXYZ(std::string path, const std::vector<Point3*> &points, ios_base::openmode mode)
	{
		if (points.size() == 0)
			return;

		std::ofstream file;
		file.open(path, mode); // std::ios::out);

		file << std::fixed << std::showpoint;
		file << std::setprecision(6);

		for (const Point3 *p : points)
		{
			file << *p << std::endl;
		}

		file.close();
	}

};