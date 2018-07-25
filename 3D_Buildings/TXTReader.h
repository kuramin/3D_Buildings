#pragma once

#include "Util.h"
#include "Point3.h"

#include <fstream>
#include <sstream>
#include <vector>

using namespace std;

class TXTReader
{

public:
	static vector<Point3> ReadPointXYZ(const string path)
	{
		vector<Point3> result;
		ifstream file(path, ios::in);

		string fileInput_point;
		vector<string> coords;

		while (file && !file.eof())
		{
			getline(file, fileInput_point);
			if (!fileInput_point.empty())
			{
				coords = util::Util::Split(fileInput_point, " ", false);
				if (coords.size() == 3)
				{
					result.push_back(Point3(atof(coords[0].c_str()), atof(coords[1].c_str()), atof(coords[2].c_str())));
				}
			}
		}

		return result;
	}

	static vector<Point3*> ReadFileToVectorOfPointers(const string path)
	{
		vector<Point3*> result;
		ifstream file(path, ios::in);

		string fileInput_point;
		vector<string> coords;

		while (file && !file.eof())
		{
			getline(file, fileInput_point);
			if (!fileInput_point.empty())
			{
				coords = util::Util::Split(fileInput_point, " ", false);
				if (coords.size() == 3)
				{
					result.push_back(new Point3(atof(coords[0].c_str()), atof(coords[1].c_str()), atof(coords[2].c_str())));
				}
			}
		}

		return result;
	}
};