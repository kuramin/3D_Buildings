#include "Sand_Ex1_funcs.h"

double dp_buffer = 1;

// function allocates new memory, creates the same structure and returns vector of pointers on it
vector<Point3*> Sand_Ex1::clone(vector<Point3*>& input) {
	vector<Point3*> result;
	for (int i = 0; i < input.size(); i++) {
		Point3* pt = new Point3();
		*pt = *input[i];
		result.push_back(pt);
	}
	return result;
}

// function returns minimal coordinates on X, Y, Z
Point3 Sand_Ex1::virt_min_xyz(const vector<Point3*>& points)
{
	Point3 result = Point3(numeric_limits<double>::max(),
		numeric_limits<double>::max(),
		numeric_limits<double>::max());

	for (const Point3* p : points)
	{
		if (p->x < result.x)
			result.x = p->x;
		if (p->y < result.y)
			result.y = p->y;
		if (p->z < result.z)
			result.z = p->z;
	}

	return result;
}

// function returns maximal coordinates on X, Y, Z
Point3 Sand_Ex1::virt_max_xyz(const vector<Point3*>& points)
{
	Point3 result = Point3(0.0, 0.0, 0.0);

	for (const Point3* p : points)
	{
		if (p->x > result.x)
			result.x = p->x;
		if (p->y > result.y)
			result.y = p->y;
		if (p->z > result.z)
			result.z = p->z;
	}

	return result;
}

// function returns a pointer to the point with minimal X (pointer to real point)
Point3* Sand_Ex1::p_real_min_x(vector<Point3*>& points)
{
	Point3* result = points[0];
	for (int i = 0; i < points.size(); i++)
	{
		if (points[i]->x < result->x) {
			result = points[i];
		}
	}

	return result;
}

// function returns a pointer to the point with maximal X (pointer to real point)
Point3* Sand_Ex1::p_real_max_x(vector<Point3*>& points)
{
	Point3* result = points[0];
	for (int i = 0; i < points.size(); i++)
	{
		if (points[i]->x > result->x) {
			result = points[i];
		}
	}

	return result;
}

// returns array of cells where each cell contains vector of corresponding points
vector<vector<vector<Point3*>>> Sand_Ex1::celled_cloud(vector<Point3*> &input, const double CELL_SIZE) {
	Point3 maxp = virt_max_xyz(input);
	int rows = floor(maxp.y / CELL_SIZE) + 1;
	int cols = floor(maxp.x / CELL_SIZE) + 1;
	vector<vector<vector<Point3*>>> result; // initialize grid

	// fill grid with empty values
	for (int i = 0; i < cols; i++) {
		result.push_back(vector<vector<Point3*>>());
		for (int j = 0; j < rows; j++) {
			result[i].push_back(vector<Point3*>());
		}
	}

	//fill grid with corresponding values
	for (Point3* p : input) {
		result[(int)(p->x / CELL_SIZE)][(int)(p->y / CELL_SIZE)].push_back(p);
	}
	return result;
}

vector<Point3*> Sand_Ex1::build_DTM_from_DSM(vector<Point3*> points_dsm, float cell_size_init, float cell_size_fin, float cell_size_step) {
	vector<Point3*> points_dtm = Sand_Ex1::clone(points_dsm);
	vector<vector<vector<Point3*>>> celled_result;
	vector<vector<Point3*>> cents;

	// iterate through filtering using descending cell size
	for (double CELL_SIZE = cell_size_init; CELL_SIZE >= cell_size_fin; CELL_SIZE = CELL_SIZE - cell_size_step) { 
		celled_result = Sand_Ex1::celled_cloud(points_dtm, CELL_SIZE); // distribute points to cells of defined size
		cents = Sand_Ex1::centroids_from_celled_cloud(celled_result); // define centroids for cells
		points_dtm = Sand_Ex1::filter_notcelled_cloud(cents, points_dtm, CELL_SIZE); // filter notcelled_result using centroids which are calculated for CELL_SIZE
	} // now filt is Digital Terrain Model of the region

	return points_dtm;
}

// returns 2D vector of points which are centroids for cells
vector<vector<Point3*>> Sand_Ex1::centroids_from_celled_cloud(vector<vector<vector<Point3*>>> &celled_cloud) {
	double z_dif;
	//vector<Point3> centroids_row;
	vector<vector<Point3*>> centroids;
	

	// fill grid with empty values
	for (int i = 0; i < celled_cloud.size(); i++) {
		centroids.push_back(vector<Point3*>());
		for (int j = 0; j < celled_cloud.at(i).size(); j++) {
			centroids[i].push_back(new Point3());
		}
	}

	for (int i = 0; i < celled_cloud.size(); i++) { // for every row of celled cloud 
		//centroids_row.clear();
		for (int j = 0; j < celled_cloud.at(i).size(); j++) {
			if (celled_cloud.at(i).at(j).size() > 0) { // for every column (every cell) of row

				Point3* nP = new Point3; // allocate new memory for centroid
				nP->x = 0; // set centroids coordinates to zeros
				nP->y = 0;
				nP->z = 0;
				for (int k = 0; k < celled_cloud.at(i).at(j).size(); k++) { // find the centroid for this cell: iterate through all points of cell
					nP->x += celled_cloud.at(i).at(j).at(k)->x;
					nP->y += celled_cloud.at(i).at(j).at(k)->y;
					nP->z += celled_cloud.at(i).at(j).at(k)->z;
				}

				nP->x = nP->x / celled_cloud.at(i).at(j).size(); // divide sums by number of points in a cell
				nP->y = nP->y / celled_cloud.at(i).at(j).size();
				nP->z = nP->z / celled_cloud.at(i).at(j).size();

				centroids[i][j] = nP;
			}
		}
	}
	return centroids;
}

// function transforms 2d array of points to a pointcloud (which is 1d array of points)
vector<Point3*> Sand_Ex1::twodim_to_onedim(vector<vector<Point3*>>& input_cloud) {
	vector<Point3*> result_cloud;
	for (int i = 0; i < input_cloud.size(); i++) { // for every row of celled cloud 
		for (int j = 0; j < input_cloud.at(i).size(); j++) {
			result_cloud.push_back(input_cloud.at(i).at(j));
		}
	}
	return result_cloud;
}

// function performs filtering of NOTcelled_cloud basing on centroids and on value of CELL_SIZE
vector<Point3*> Sand_Ex1::filter_notcelled_cloud(vector<vector<Point3*>> &centroids, vector<Point3*> &cloud, const double CELL_SIZE) {
	vector<Point3*> result_cloud;
	//Point3* nP;
	double alpha;
	double z_dif;
	int i, j;
	for (int k = 0; k < cloud.size(); k++) { // for every point of notcelled cloud

		// define i and j to choose appropriate centroid
		i = (int)(cloud[k]->x / CELL_SIZE);
		j = (int)(cloud[k]->y / CELL_SIZE);
		if (cloud[k]->z <= centroids.at(i).at(j)->z) { // if point is lower than reference plane
			alpha = 1.0;
		}
		else if (cloud[k]->z > centroids.at(i).at(j)->z + 1.0) { // if point is too high above reference plane
			alpha = 0;
		}
		else { // if point is above reference plane but not too much
			z_dif = cloud[k]->z - centroids.at(i).at(j)->z;
			alpha = 1 / (1 + pow(z_dif, 4)); // alpha -> 1 for points a bit higher than ref, alpha -> 1/2 for points 1m above ref
		}

		Point3* nP = new Point3; // allocate new memory for new member of result_cloud, which corresponds to point cloud[k]
		*nP = *cloud[k]; // new point is set to be similar with point from celled cloud
		nP->z = (1 - alpha) * centroids.at(i).at(j)->z + alpha * cloud[k]->z; // but its z is fixed using centoids z
		result_cloud.push_back(nP); // new point is pushed_back to result cloud
	}
	return result_cloud;
}

// function builds a Digital Elevation Model(DEM) as a difference of Digital Surface Model(DSM) and Digital Terrain Model(DTM)
vector<Point3*> Sand_Ex1::points_above_DTM(const vector<Point3*> &points_dsm, const vector<Point3*> &points_dtm, vector<Point3*> &points_ground, const double offset = 0.0, const bool drop_to_sealevel = 1)
{
	vector<Point3*> result;
	Point3* nP;
	for (int i = 0; i < points_dsm.size(); i++) {  // walk through both vectors
		if (points_dsm[i]->z - points_dtm[i]->z > offset) { // if for current point z difference is more than offset (point is on building)
			nP = points_dsm[i]; // then take x,y values from current point
			//if (drop_to_sealevel) nP->z = points_dsm[i]->z - points_dtm[i]->z; // take z value as difference of z values
			//else nP->z = points_dsm[i]->z;
			result.push_back(nP); // and put this new point into result vector
		}
		else {
			nP = points_dsm[i]; // then take x,y values from current point
			//if (drop_to_sealevel) nP->z = points_dsm[i]->z - points_dtm[i]->z; // take z value as difference of z values
			//else nP->z = points_dsm[i]->z;
			points_ground.push_back(nP); // and put this new point into result vector
		}
	}
	return result;
}

// fuction returns vector of those points which have 3 or more neighbors closer than rad3D distance to them
vector<Point3*> Sand_Ex1::density_threshold(vector<Point3*> &points, double rad3D, int numPoints) { // 25250, 25427
	vector<Point3*> result;
	double dist_sq;
	int counter;
	for (int i = 0; i < points.size(); i++) { // for every pint i of the cloud
		counter = 0;
		for (int j = 0; j < points.size(); j++) { // for every point j of the same cloud
			if (i != j) { // if points i and j are not the same, then calculate distance between them
				dist_sq = pow(points[i]->x - points[j]->x, 2) + pow(points[i]->y - points[j]->y, 2) + pow(points[i]->z - points[j]->z, 2);
				if (dist_sq <= pow(rad3D, 2)) { // if distance between point j and point i is less than rad3D, then
					counter++; // increment counter
					if (counter > 2) { // and if point i has now more than 2 close neighboors, push it back to result and move to next point i
						result.push_back(points[i]);
						break;
					}
				}
			}
		}
	}
	return result;
}

// function establishes neighborhood relations in the pointcloud based on distance radius2D
void Sand_Ex1::determine_neighborhood(vector<Point3*> &points, const double radius2D)
{
	for (unsigned int i = 0; i < points.size() - 1; i++)
	{
		//cout << "Neigh for point " << i << endl;
		for (unsigned int j = i + 1; j < points.size(); j++)
		{
			if ((*points[i] - *points[j]).Length2D() <= radius2D) // Length2D
			{
				points[i]->points_neighbor.push_back(points[j]);
				points[j]->points_neighbor.push_back(points[i]);
			}
		}
	}
}

// function determines next point of outline detection for current_line (based on intrinsic neighborhood relations)
Point3* Sand_Ex1::determine_next_point(vector<Point3*>& current_line) {
	double max_ang = 0;
	double ang;
	bool intersects;
	Point3* cur_p = current_line[current_line.size() - 1]; // let the last point of current vector be the current point
	Point3* prev_p = current_line[current_line.size() - 2]; // let the prelast point be the previous point
	Point3* outline_point = current_line[current_line.size() - 3]; // let the pre-pre-last point be the first to consider "outline point"
	Point3* next_p_candid; // candidates for next point
	Point3* next_p_fin = cur_p; // final next point
	Vector3 vec_to_prev = Vector3(prev_p->x - cur_p->x, prev_p->y - cur_p->y, prev_p->z - cur_p->z); // vector from current point to previous point
	Vector3 vec_to_next;
	bool isec = 0;

	//cout << endl << "Current point is " << cur_p->z << endl;
	// iterate through cur_p.points_neighbors
	for (int i = 0; i < cur_p->points_neighbor.size(); i++) {
		next_p_candid = cur_p->points_neighbor[i]; // choose i-th neighbor as a candidate next point
		vec_to_next = Vector3(next_p_candid->x - cur_p->x, next_p_candid->y - cur_p->y, next_p_candid->z - cur_p->z); // vector from current point to candidate
		ang = Angle2DDegree360(vec_to_prev, vec_to_next); // calculate angle between vectors
														  //cout << "    Angle from point " << cur_p->z << " to point " << next_p_candid->z << " is " << ang << ". Current max_ang is " << max_ang << ". Now lets check intersection" << endl;

														  // this i-th neighbor of current point has chance to become a "final next" if new angle > max_ang and ...
														  // ... and segment (cur_p - next_p_candid) doesnt intersect any segment (outline_point[i] - outline_point[i+1]) 
														  // for i from 2 (first 2 points are virtual) to [current_line.size() - 3] (when [i+1] is the previous point which is [current_line.size() - 2]).
														  // So lets iterate through segments and if we dont find any intersections, bool "intersects" will remain false
		intersects = 0;
		for (int j = 2; j < current_line.size() - 2; j++) { // for all segments from 2 to current_line.size() - 3
															/*if (j == 323)
															cout << "jjjjjj = 323" << endl;*/
			isec = Intersection::InnerIntersection2DLineSegments(*current_line[j], *current_line[j + 1], *cur_p, *next_p_candid);
			if (isec) { //Intersection::InnerIntersection2DLineSegments(*current_line[j], *current_line[j + 1], *cur_p, *next_p_candid)) { // if intersects
				intersects = 1; // set bool to 1 and get out from for loop
								//if (current_line.size() > 315 && current_line.size() < 340) //del
								//cout << "Point " << next_p_candid->z << " cannot be next after " << cur_p->z << " because " << endl; //del
								//cout << "        (" << current_line[j]->z << ") - (" << current_line[j + 1]->z << ") intersects (" << cur_p->z << ") - (" << next_p_candid->z << ")" << " here j is " << j << endl;
				break;
			}
		}


		if (ang >= max_ang && !intersects) { // if ang > max_ang and there is no intersection
			max_ang = ang; // update max_ang and make this point final next
			next_p_fin = next_p_candid;
			//cout << "    Point " << next_p_candid->z << " is chosen to be next after " << cur_p->z << ". Ang is " << ang << endl << endl;
		}
	}

	return next_p_fin;
}

// function builds a modified convex hull of vec_building
vector<Point3*> Sand_Ex1::outline_by_mod_convhull(vector<Point3*> vec_building) {
	Point3* next_p;
	Point3* cur_p = p_real_min_x(vec_building);
	Point3* prev_p = &Point3(cur_p->x, cur_p->y - 100, cur_p->z); // point exactly on south from current will be previous          -------- или через New?
	Point3* preprev_p = prev_p; // and pre-previous

	vector<Point3*> outline;
	outline.push_back(preprev_p); // starting vector will consist of these three points
	outline.push_back(prev_p);
	outline.push_back(cur_p);
	//cout << cur_p->x << " " << cur_p->y << " " << cur_p->z << " is starting point" << endl;
	do { // perform steps
		next_p = determine_next_point(outline);
		//if (outline.size() > 315) //del
		//cout << next_p->x << " is defined as next and pushed into line of " << outline.size() << endl; //del
		outline.push_back(next_p); // push it
	} while (next_p != outline[2]); // while starting point not found again   // del  && outline.size() < 340

	outline.erase(outline.begin()); // erase 2 front points, because they were used to start and are not real
	outline.erase(outline.begin());
	return outline;
}

// returns part of vector with first index and last index defined
vector<Point3*> Sand_Ex1::part_of_vector(vector<Point3*> input, int first, int last) {
	vector<Point3*> result;
	for (int k = first; k < last + 1; k++) {
		result.push_back(input[k]);
	}
	return result;
}

// returns vector-result of DouglasPeucker with buffer epsilon to a polyline
vector<Point3*> Sand_Ex1::douglas_peucker(vector<Point3*> polyline, double epsilon) {
	double dmax = 0.0;
	int index = 0;
	int length = polyline.size();
	vector<Point3*> part1, part2;
	vector<Point3*> recResults1, recResults2;
	vector<Point3*> result; // result
	if (length > 2) { // if polyline is longer than 2
					  // lets find index of the furtherst points and its distance
		for (int i = 1; i < length - 1; i++) { // for every point except the first and the last
											   // calculate distance from point to (first - last) connecting line
			double d = perpendicularDistancePointToLine(*polyline[i], lineCoefficients(*polyline[0], *polyline[length - 1]));
			if (d > dmax) { // if this distance is more than current maximum, update maximum
				index = i;
				dmax = d;
			}
		}

		// if found maximum distance is more than epsilon
		if (dmax >= epsilon) {
			part1 = part_of_vector(polyline, 0, index); // part1 - part of polyline from beginning to the furtherst point
			recResults1 = douglas_peucker(part1, epsilon); // recResult1 - result of DouglasPeucker for part1
			part2 = part_of_vector(polyline, index, length - 1); // part2 - part of polyline from the furtherst point to the end
			recResults2 = douglas_peucker(part2, epsilon); // recResult2 - result of DouglasPeucker for part2
			result = part_of_vector(recResults1, 0, recResults1.size() - 2); // result consists of recResults1 points from beginning till prelast
			for (int k = 0; k < recResults2.size(); k++) { // with addition of all recResults2 points
				result.push_back(recResults2[k]);
			}
		}
		else { // if no any points falls away from buffer
			result.push_back(polyline[0]); // then result is built from the first and the last points
			result.push_back(polyline[length - 1]);
		}
	}
	else { // if polyline consists of 2 points or less
		result = polyline; // then result is polyline itself
	}

	return result;
}

// returns vector - result of DouglasPeucker application to a closed polygon outline
vector<Point3*> Sand_Ex1::douglas_peucker_of_closed_polygon(vector<Point3*> outline) {

	int ind_min_x, ind_max_x;
	Point3* p_min_x = p_real_min_x(outline); // find point with minimal X
	Point3* p_max_x = p_real_max_x(outline); // find point with maximal X
	for (int k = 0; k < outline.size(); k++) { // find corresponding indexes
		if (outline[k] == p_min_x) ind_min_x = k;
		if (outline[k] == p_max_x) ind_max_x = k;
	}

	// since the starting point can be above min-max line or below, we need to consider index which comes first and which comes second
	int first_ind = min(ind_min_x, ind_max_x);
	int second_ind = max(ind_min_x, ind_max_x);

	vector<Point3*> outline_part1 = part_of_vector(outline, 0, first_ind); // is a part of outline from index 0 to min(ind_min_x, ind_max_x)
	vector<Point3*> outline_part2 = part_of_vector(outline, first_ind, second_ind); // is a part of outline from index min(ind_min_x, ind_max_x) to index max(ind_min_x, ind_max_x)
	vector<Point3*> outline_part3 = part_of_vector(outline, second_ind, outline.size() - 1); // is a part of outline from index max(ind_min_x, ind_max_x) to the end
	if (first_ind != 0) { // all points from outline_part1 except 0 are added into outline_part3
		for (int k = 1; k < outline_part1.size(); k++) {
			outline_part3.push_back(outline_part1[k]);
		}
	}

	// simplified outline from index min(ind_min_x, ind_max_x) to index max(ind_min_x, ind_max_x) is
	vector<Point3*> dougpeu = douglas_peucker(outline_part2, dp_buffer);
	// is a simplified outline from index max(ind_min_x, ind_max_x) to index min(ind_min_x, ind_max_x) coming through index 0 is
	vector<Point3*> dougpeu_addition = douglas_peucker(outline_part3, dp_buffer);
	for (int k = 1; k < dougpeu_addition.size(); k++) { // dougpeu of outline_part3 is added to dougpeu of outline_part2
		dougpeu.push_back(dougpeu_addition[k]);
	}

	return dougpeu;
}

// clusterises pointcloud by pointers
vector<vector<Point3*>> Sand_Ex1::clusterize_point_cloud_to_vec_vec_pointers(vector<Point3*>& all_pts) { // the question is pointers or not pointers
	vector<vector<Point3*>> cluster_array; // here we will store our result
	list<Point3*> queue; // queue which is used for breadth first search
	int total = 0;
	Point3 p, n;
	Point3* p_p;
	int clus = 0; // index of cluster

	for (int i = 0; i < all_pts.size(); i++) { // for all points of cloud
		if (all_pts[i]->visited == 0) { // when we find a non-visited point
			cluster_array.push_back(vector<Point3*>()); // we create a new vector in cluster_array - new container for new cluster
			queue.push_back(all_pts[i]); // we push a pointer to this point into the queue. Now lets find a cluster for this point
			while (!queue.empty()) {
				p_p = queue.front();
				queue.pop_front();
				cluster_array[clus].push_back(p_p); // and push the reference to a copy back to the cluster with index [clus]
				p_p->visited = true; // mark copy-point as visited
				for (int s = 0; s < p_p->points_neighbor.size(); s++) {
					if (p_p->points_neighbor[s]->visited == 0) {
						queue.push_back(p_p->points_neighbor[s]);
						p_p->points_neighbor[s]->visited = true;
					}
				}
			}
			//cout << "  Cluster number " << clus << " got " << cluster_array[clus].size() << " points" << endl;
			total += cluster_array[clus].size();
			clus++;
		}
	}

	//Set points of result as unvisited
	for (int o = 0; o < cluster_array.size(); o++) {
		Sand_Ex1::set_points_unvisited(cluster_array[o]);
		}

	return cluster_array;
}

// builds a vector of flat-roof buildings from clusterised pointcloud
vector<Building3D*> Sand_Ex1::vector_of_flatroofs_from_clusterised_point_cloud(vector<vector<Point3*>>& cluster_array, vector<vector<Point3*>>& outl_array, vector<vector<Point3*>>& dgp_array) {
	
	vector<Building3D*> vec_of_flats;
	vector<Poly3D*> vec_of_pols;
	Poly3D* pol;
	ios_base::openmode open_mode = ofstream::trunc;
	vector<Point3*> outline, dougpeu;
	vector<Point3*> building_points;
	Building3D* flatroof;
	float mean_height;
	
	for (int i = 0; i < cluster_array.size(); i++) { // for every cluster
		building_points = cluster_array[i]; // let building_points be vector of pointers to points of current building

		if (building_points.size() < 10) continue; 								   
		outline = outline_by_mod_convhull(building_points); // outline of cloud points_building
		outl_array.push_back(outline);

		dougpeu = douglas_peucker_of_closed_polygon(outline); // find DouglasPeucker of polygon
		if (dougpeu.size() < 4) continue;
		dgp_array.push_back(dougpeu);

		flatroof = new Building3D();
		pol = new Poly3D();
	
		for (int s = 0; s < dougpeu.size(); s++) { // create outer contour from points of found douglas-peucker
			pol->outer_contour.points.push_back(*dougpeu[s]);
		}
		flatroof->polygon = pol;

		mean_height = 0.0;
		for (Point3* p : building_points) 
			mean_height += p->z;

		flatroof->eaves_height = mean_height / building_points.size();
		flatroof->foot_height = flatroof->eaves_height;

		vec_of_flats.push_back(flatroof);		
	}

	return vec_of_flats; // global variable
}

float Sand_Ex1::reduce_heights(vector<Building3D*> vec_of_flats) {
	// find minimal foot_height
	float min_foot_hei = FLT_MAX;
	for (int k = 0; k < vec_of_flats.size(); k++) {
		if (vec_of_flats[k]->foot_height < min_foot_hei)
			min_foot_hei = vec_of_flats[k]->foot_height;
	}
	// for every Building3D subtract min_foot_height from foot_height
	for (int k = 0; k < vec_of_flats.size(); k++) {
		vec_of_flats[k]->foot_height -= min_foot_hei;
		vec_of_flats[k]->eaves_height -= min_foot_hei;
	}
	return min_foot_hei;
}

void Sand_Ex1::reduce_cloud_heights(vector<Point3*> cloud, float value) {
	// for every point of a cloud subtract min_foot_height from its z value
	for (int k = 0; k < cloud.size(); k++) {
		cloud[k]->z -= value;
	}
}

// deletes singlefriended vertices from each cluster
void Sand_Ex1::delete_singlefriended_vertices(vector<vector<Point3*>>& cluster_array) {
	Point3* p;
	Point3* n;
	for (int i = 0; i < cluster_array.size(); i++) { // for each cluster of cluster_array
		for (int j = 0; j < cluster_array[i].size(); j++) { // for each vertex of cluster
															//cout << " point cell is " << &cluster_array[i][j] << endl;
			p = cluster_array[i][j];
			if (p->points_neighbor.size() == 1) {  // if vertex p has only 1 neighbor

   			    // go to this neighbor n and delete connection to point p:
				n = p->points_neighbor[0];
				for (int k = 0; k < n->points_neighbor.size(); k++) { // for every neighbor of n
					
					// if this neighbor of n is vertex p, 												 
					if (n->points_neighbor[k] == p) {
						cout << "  Singlefriended found and deleted from cluster " << i << " of size " << cluster_array[i].size() << endl;
						// delete neighborhood relation of n to p - erase from vector of pointers n.points_neighbor the member number k, not to null, but erase
						n->points_neighbor.erase(n->points_neighbor.begin() + k); // by the way all the rest of vector doesnot shift 1 step forward, all pointers stay in the same memory cells
																				  // delete neighborhood relation of p to n
						p->points_neighbor.erase(p->points_neighbor.begin());
						// delete vertex p itself
						cluster_array[i].erase(cluster_array[i].begin() + j);
						// jump out from for-loop
						break;
					}

				}
			}
		}
	}
}

// move point cloud by pointers to points
void Sand_Ex1::move_points(vector<Point3*>& vec, Point3 offset) {
	for (Point3* &p : vec)
	{
		if (!p->visited) {
			p->x -= offset.x;
			p->y -= offset.y;
			p->visited = 1;
		}
	}

	for (Point3* &p : vec)
	{
		p->visited = 0;
	}
}

// set point cloud unvisited by pointers to points
void Sand_Ex1::set_points_unvisited(vector<Point3*>& vec) {
	for (Point3* &p : vec)
	{
		p->visited = 0;
	}
}


// function saves coefficients A, B, C, D of plane through points p1, p2, p3
plane_coef Sand_Ex1::planeCoefficients(Point3* p1, Point3* p2, Point3* p3) {
	plane_coef result;
	result.A = p1->y * p2->z + p3->y * p1->z + p2->y * p3->z - p3->y * p2->z - p2->y * p1->z - p1->y * p3->z;
	result.B = -(p1->x * p2->z + p3->x * p1->z + p2->x * p3->z - p3->x * p2->z - p2->x * p1->z - p1->x * p3->z);
	result.C = p1->x * p2->y + p3->x * p1->y + p2->x * p3->y - p3->x * p2->y - p2->x * p1->y - p1->x * p3->y;
	result.D = -(p1->x * p2->y * p3->z + p3->x * p1->y * p2->z + p2->x * p3->y * p1->z - p3->x * p2->y * p1->z - p2->x * p1->y * p3->z - p1->x * p3->y * p2->z);
	return result;
}

// calculates distance from point to the plane given by A, B, C, D
float Sand_Ex1::perpendicularDistancePointToPlane(Point3* point, plane_coef pl_c) {
	return abs((pl_c.A * point->x + pl_c.B * point->y + pl_c.C * point->z + pl_c.D) / sqrt(pow(pl_c.A, 2) + pow(pl_c.B, 2) + pow(pl_c.C, 2)));
}


// function returns coefficients a, b, c of line through begin_point and end_point in a Point3-format
Point3 Sand_Ex1::lineCoefficients(Point3 begin_point, Point3 end_point) {
	return Point3(end_point.y - begin_point.y, begin_point.x - end_point.x, begin_point.y * end_point.x - begin_point.x * end_point.y);
}

// calculates distance from point to the line given by a,b,c in coef_abc
double Sand_Ex1::perpendicularDistancePointToLine(Point3 point, Point3 coef_abc) {
	return abs(coef_abc.x * point.x + coef_abc.y * point.y + coef_abc.z) / sqrt(pow(coef_abc.x, 2) + pow(coef_abc.y, 2));
}

// function deletes those members of vec of vec of Point3*, which are supposed to be vegetation based on plane of neighborhood
void Sand_Ex1::separate_vegetation(vector<vector<Point3*>>& input, vector<Point3*>& points_veget, float thresh_hei_dif) {
	float sum_hei_dif;
	int i = 0;

	while (i < input.size()) {
		sum_hei_dif = 0;
		for (int j = 0; j < input[i].size(); j++) { // for every point of a cluster

			// increment sum_hei_dif by distance from point to the plane, which is based on neighbors of current point
			if (input[i][j]->points_neighbor.size() > 2)
				sum_hei_dif += distance_from_point_to_plane_based_on_neighbors(input[i][j]);

		} // now we have value of sum_hei_dif for current cluster
		sum_hei_dif = sum_hei_dif / input[i].size(); // normalize sum_hei_dif by cluster size

		if (sum_hei_dif > thresh_hei_dif) {
			for (int j = 0; j < input[i].size(); j++) {
				points_veget.push_back(input[i][j]);
			}
			input.erase(input.begin() + i);
		}
		else
			i++;
	}
}

// function deletes those members of vec<vec<Point3*>> which have size less than thresh_clus_size
void Sand_Ex1::delete_small_clusters(vector<vector<Point3*>>& input, int thresh_clus_size) {
	int i = 0;
	// while on the position i there is vector of size less than threshold, delete vector on position i
	while (i < input.size())
	{
		if (input[i].size() < thresh_clus_size) // if element must be deleted
			input.erase(input.begin() + i); // do it
		else
			i++; // otherwise go to next index
	}
}

// function calculates optimal plane for points neighborhood and returns distance from point to this plane 
float Sand_Ex1::distance_from_point_to_plane_based_on_neighbors(Point3* point) {
	float dist, sum_dist = numeric_limits<float>::max();
	plane_coef pl_c, best_pl_c;
	int num_nei = point->points_neighbor.size();
	if (num_nei > 2) {
		for (int i = 0; i < num_nei; i++) { // for every first point
			for (int j = i + 1; j < num_nei; j++) { // for every second point
				for (int k = j + 1; k < num_nei; k++) { // for every third point

					pl_c = planeCoefficients(point->points_neighbor[i], point->points_neighbor[j], point->points_neighbor[k]); // calculate plane coefs of trinity
					dist = 0; // set dist to 0
					for (int m = 0; m < num_nei; m++) { // for every neighbor of current point 
						dist += Sand_Ex1::perpendicularDistancePointToPlane(point->points_neighbor[m], pl_c);
					} // now we have summated distance for current trinity

					if (dist < sum_dist) {
						sum_dist = dist;
						best_pl_c = pl_c;
					}
				}
			}
		} // now we know best plane for current point neighbors
		return Sand_Ex1::perpendicularDistancePointToPlane(point, best_pl_c); // return distance from current point to the best plane
	}
	else {
		cout << "Point has less than 3 neighbors" << endl;
		return 0;
	}
}