#include <iostream>
#include <list>
#include "Point3.h"
#include "TXTReader.h"
#include "TXTWriter.h"
#include "Intersection.h"
#include "Building3D.h"

using namespace std;
struct plane_coef {
	float A;
	float B;
	float C;
	float D;
};

class Sand_Ex1
{
	public:
		// constructor 
		Sand_Ex1(void) {};

		// destructor 
		~Sand_Ex1(void) {};

		// function allocates new memory, creates the same structure and returns vector of pointers on it
		vector<Point3*> clone(vector<Point3*>&);

		// function returns minimal coordinates on X, Y, Z
		Point3 virt_min_xyz(const vector<Point3>&);

		// function returns minimal coordinates on X, Y, Z
		Point3 virt_min_xyz(const vector<Point3*>&);

		// function returns maximal coordinates on X, Y, Z
		Point3 virt_max_xyz(const vector<Point3>&);

		// function returns maximal coordinates on X, Y, Z
		Point3 virt_max_xyz(const vector<Point3*>&);

		// function returns a pointer to the point with minimal X (pointer to real point)
		Point3* p_real_min_x(vector<Point3*>&);

		// function returns a pointer to the point with maximal X (pointer to real point)
		Point3* p_real_max_x(vector<Point3*>&);

		// returns array of cells where each cell contains vector of corresponding points
		vector<vector<vector<Point3*>>> celled_cloud(vector<Point3*>&, const double);

		// function builds DTM from DSM by filtering
		vector<Point3*> build_DTM_from_DSM(vector<Point3*>, float, float, float);

		// returns 2D vector of points which are centroids for cells
		vector<vector<Point3>> centroids_from_celled_cloud(vector<vector<vector<Point3>>>&);

		// returns 2D vector of points which are centroids for cells
		vector<vector<Point3*>> centroids_from_celled_cloud(vector<vector<vector<Point3*>>>&);

		// function transforms 2d array of points to a pointcloud (which is 1d array of points)
		vector<Point3> twodim_to_onedim(vector<vector<Point3>>&);

		// function transforms 2d array of points to a pointcloud (which is 1d array of points)
		vector<Point3*> twodim_to_onedim(vector<vector<Point3*>>&);

		// function performs filtering of NOTcelled_cloud basing on centroids and on value of CELL_SIZE
		vector<Point3> filter_notcelled_cloud(vector<vector<Point3>>&, vector<Point3>&, const double);

		// function performs filtering of NOTcelled_cloud basing on centroids and on value of CELL_SIZE
		vector<Point3*> filter_notcelled_cloud(vector<vector<Point3*>>&, vector<Point3*>&, const double);

		// function builds a Digital Elevation Model(DEM) as a difference of Digital Surface Model(DSM) and Digital Terrain Model(DTM)
		vector<Point3*> points_above_DTM(const vector<Point3*>&, const vector<Point3*>&, vector<Point3*>&, const double, const bool);

		// fuction returns vector of those points which have 3 or more neighbors closer than rad3D distance to them
		vector<Point3> density_threshold(vector<Point3>&, double, int);

		// fuction returns vector of those points which have 3 or more neighbors closer than rad3D distance to them
		vector<Point3*> density_threshold(vector<Point3*>&, double, int);

		// returns part of vector with first index and last index defined
		vector<Point3> part_of_vector(vector<Point3>, int, int);

		// function establishes neighborhood relations in the pointcloud based on distance radius2D
		void determine_neighborhood(vector<Point3>&, const double);

		// function establishes neighborhood relations in the pointcloud based on distance radius2D
		void determine_neighborhood(vector<Point3*>&, const double);

		// function determines next point of outline detection for current_line (based on intrinsic neighborhood relations)
		Point3* determine_next_point(vector<Point3*>&);

		// function builds a modified convex hull of vec_building
		vector<Point3*> outline_by_mod_convhull(vector<Point3*>);

		// returns part of vector with first index and last index defined
		vector<Point3*> part_of_vector(vector<Point3*>, int, int);

		// returns vector-result of DouglasPeucker with buffer epsilon to a polyline
		vector<Point3*> douglas_peucker(vector<Point3*>, double);

		// returns vector - result of DouglasPeucker application to a closed polygon outline
		vector<Point3*> douglas_peucker_of_closed_polygon(vector<Point3*>);

		// clusterises pointcloud by pointers
		vector<vector<Point3*>> clusterize_point_cloud_to_vec_vec_pointers(vector<Point3>&);

		// clusterises pointcloud by pointers
		vector<vector<Point3*>> clusterize_point_cloud_to_vec_vec_pointers(vector<Point3*>&);

		// builds a vector of flat-roof buildings from clusterised pointcloud
		vector<Building3D*> vector_of_flatroofs_from_clusterised_point_cloud(vector<vector<Point3*>>&, vector<vector<Point3*>>&, vector<vector<Point3*>>&);

		// function reduces height of vector of flatroofs closer to 0, so that it could be visualized, and returns the value of reduction 
		float reduce_heights(vector<Building3D*>);

		// function reduces height of vector of points by value 
		void reduce_cloud_heights(vector<Point3*>, float);

		// deletes singlefriended vertices from each cluster
		void delete_singlefriended_vertices(vector<vector<Point3*>>&);

		// move point cloud by points
		void move_points(vector<Point3>&, Point3);

		// move point cloud by pointers to points
		void move_points(vector<Point3*>&, Point3);

		// set point cloud unvisited by points
		void set_points_unvisited(vector<Point3>&);

		// set point cloud unvisited by pointers to points
		void set_points_unvisited(vector<Point3*>&);

		// function saves coefficients A, B, C, D of plane through points p1, p2, p3
		plane_coef planeCoefficients(Point3*, Point3*, Point3*);

		// calculates distance from point to the plane given by A, B, C, D
		float perpendicularDistancePointToPlane(Point3*, plane_coef);

		// function returns coefficients a, b, c of line through begin_point and end_point in a Point3-format
		Point3 lineCoefficients(Point3, Point3);

		// calculates distance from point to the line given by a,b,c in coef_abc
		double perpendicularDistancePointToLine(Point3, Point3);

		// function deletes those members of vec of vec of Point3*, which are supposed to be vegetation based on plane of neighborhood
		void separate_vegetation(vector<vector<Point3*>>&, vector<Point3*>&, float);

		// function deletes those members of vec<vec<Point3*>> which have size less than thresh_clus_size
		void delete_small_clusters(vector<vector<Point3*>>&, int);

		// function calculates optimal plane for points neighborhood and returns distance from point to this plane 
		float distance_from_point_to_plane_based_on_neighbors(Point3*);
};