#include "Sand_Ex1_funcs.h"
#include <GL/glut.h> 

// Constants
vector<Building3D*> vec_of_buildings;
vector<Point3*> points_dsm;
vector<Point3*> points_dsm_copy;
vector<Point3*> points_dtm;
vector<Point3*> points_above_ground_copy;
vector<Point3*> points_ground;
vector<Point3*> points_veget;
vector<vector<Point3*>> cluster_array;
vector<vector<Point3*>> outl_array;
vector<vector<Point3*>> dgp_array;

// OpenGl constants
static GLfloat BLACK[] = { 0.0f, 0.0f, 0.0f };
static GLfloat RED[] = { 1.0f, 0.0f, 0.0f };
static GLfloat GREEN[] = { 0.0f, 1.0f, 0.0f };
static GLfloat BLUE[] = { 0.0f, 0.0f, 1.0f };
static GLfloat YELLOW[] = { 1.0f, 1.0f, 0.0f };
static GLfloat DARK_YELLOW[] = { 0.5f, 0.5f, 0.0f };
static GLfloat PURPLE[] = { 1.0f, 0.0f, 1.0f };
static GLfloat CYAN[] = { 0.0f, 1.0f, 1.0f };
static GLfloat DARK_CYAN[] = { 0.0f, 0.5f, 0.5f };
static GLfloat ORANGE[] = { 1.0f, 0.5f, 0.0f };
static GLfloat LIGHT_GREEN[] = { 0.5f, 1.0f, 0.5f };
static GLfloat DARK_GREEN[] = { 0.0f, 0.5f, 0.0f };
static GLfloat BROWN[] = { 0.5f, 0.25f, 0.0f };
static GLfloat WHEAT[] = { 0.85f, 0.85f, 0.55f };

GLfloat theta_x = 0.0;
GLfloat theta_y = 0.0;
GLfloat shift_x = 0.0;
GLfloat shift_y = 0.0;
GLfloat scale = 1.0;
GLfloat height_factor = 4.0;
int mouse_button_event_pix_x = 0;
int mouse_button_event_pix_y = 0;
bool left_button = false;
bool right_button = false;
bool middle_button = false;
GLfloat window_pix_width, window_pix_height;
GLfloat max_coord = 250;
bool on_1 = 0, on_2 = 0, on_3 = 0, on_4 = 0, on_5 = 0, on_6 = 0, on_7 = 0, on_8 = 0, on_9 = 0, on_0 = 0;
vector<Point3> tess_pts;

//------------------------------------------OPENGL functions ----------------------------------

void CALLBACK beginCallback(GLenum which) {} // is called to start transmission of next element (analog of glBegin)

void CALLBACK endCallback(void) {} // is called when element is transmitted (analog of glEnd)

void CALLBACK TessEdgeFlagCallback(GLboolean flag) {}

// is called to transmit next vertex (as a pointer which is given to tesselator), (analog of glVertex)
void CALLBACK vertexCallback(void* vertex_data) {
	double* xyz = (double*)vertex_data;
	Point3 pt(xyz[0], xyz[1], xyz[2]);
	tess_pts.push_back(pt);
}

// function draws tesselated figure (set of triangles from global variable)
void DrawTesselated(float height, GLfloat* color) {
	glBegin(GL_TRIANGLES); // draw triangle
	glColor3fv(color); // set drawing color
	for (Point3 p : tess_pts) {
		glVertex3f(p.x, p.y, height);
	}
	glEnd();
}

// function performs tesselation (here - triangulation) of polygon poly_pointer and store result into global variable tess_pts
void tesselationOfPolygonWithHoles(Poly3D* poly_pointer) {
	// clear container
	tess_pts.clear();

	// convert polygon into 1D array of doubles - format which is accepted for tesselator
	int pts_no = poly_pointer->outer_contour.points.size(); // number of points in input polygon
	for (Contour3D contour : poly_pointer->inner_contours) {
		pts_no += contour.points.size();
	}

	double* pPts = new double[3 * pts_no]; // pPts is a pointer to 1D array of [x1, y1, z1, z2...] format
	double* pPoints = pPts; // pPoints is equal to pPts

	for (Point3 p : poly_pointer->outer_contour.points) {
		*pPoints++ = p.x; // fill an array with triples of x,y,z
		*pPoints++ = p.y;
		*pPoints++ = 0;
	}

	for (Contour3D contour : poly_pointer->inner_contours) {
		for (Point3 p : contour.points) {
			*pPoints++ = p.x; // fill an array with triples of x,y,z
			*pPoints++ = p.y;
			*pPoints++ = 0;
		}
	}

	pPoints = pPts; // return pointer pPoints back to the beginning of the array
	GLUtesselator *tess = gluNewTess(); // create new tesselation object and return pointer to it (antipod of gluDeleteTess)

	gluTessCallback(tess, GLU_TESS_BEGIN, (void(CALLBACK*) ()) &beginCallback); // register tesselation callbacks
	gluTessCallback(tess, GLU_TESS_END, (void(CALLBACK*) ()) &endCallback);
	gluTessCallback(tess, GLU_TESS_VERTEX, (void(CALLBACK*) ()) &vertexCallback);
	gluTessCallback(tess, GLU_TESS_EDGE_FLAG, (void(CALLBACK*) ()) &TessEdgeFlagCallback);
	gluTessBeginPolygon(tess, 0); // start polygon definition, consists of 1 or more TessContours
	gluTessBeginContour(tess); // start contour definition, consists of 0 or more TessVertex

	// define contours (for every point of input polygon = for every triple of aray)
	for (int i = 0; i < poly_pointer->outer_contour.points.size(); i++, pPoints += 3) {
		gluTessVertex(tess, pPoints, pPoints);
	}

	gluTessEndContour(tess); // close contour definition

	for (Contour3D contour : poly_pointer->inner_contours) { // for every contour of inner_contour
		gluTessBeginContour(tess); // start contour definition, consists of 0 or more TessVertex
		for (int i = 0; i < contour.points.size(); i++, pPoints += 3) { // define contours (for every point of input polygon = for every triple of aray)
			gluTessVertex(tess, pPoints, pPoints);
		}
		gluTessEndContour(tess); // close contour definition
	}
	gluTessEndPolygon(tess); // close polygon tesselation	
	gluDeleteTess(tess); // delete tesselation object
}

// function finds center and half-range of vector of Poly3Ds (given by pointers)
vector<Point3> Median_and_range_of_vector(vector<Point3*>& input) {
	Point3 min(FLT_MAX, FLT_MAX, 0);
	Point3 max(FLT_MIN, FLT_MIN, 0);
	for (const Point3* p : input) {
		if (p->x > max.x)
			max.x = p->x;
		if (p->x < min.x)
			min.x = p->x;
		if (p->y > max.y)
			max.y = p->y;
		if (p->y < min.y)
			min.y = p->y;
	}

	vector<Point3> result(2);
	result[0].x = (max.x + min.x) / 2;
	result[0].y = (max.y + min.y) / 2;
	result[1].x = (max.x - min.x) / 2;
	result[1].y = (max.y - min.y) / 2;
	return result;
}

// draws all point-object from specified parameters
void DrawAllPointFeatures(vector<Point3*> vec_of_pts, GLfloat* color, int pointsize) {
	glPointSize(pointsize);
	glColor3fv(color); // set drawing color
	glBegin(GL_POINTS);
	for (const Point3* p : vec_of_pts) {
		glVertex3f(p->x, p->y, height_factor * p->z);
	}
	glEnd();
}

// draws polygon on given height, used to draw footprint or eavesline
void DrawPolygonOnSpecifiedHeight(Poly3D* polygon, GLfloat height) {
	glBegin(GL_LINE_STRIP); // draw outer contour
	for (Point3 p : polygon->outer_contour.points) {
		glVertex3f(p.x, p.y, height);
	}
	glEnd();

	for (Contour3D c : polygon->inner_contours) { // draw each inner contour
		glBegin(GL_LINE_STRIP);
		for (Point3 p : c.points) {
			glVertex3f(p.x, p.y, height);
		}
		glEnd();
	}
}

// draws all polygon object from specified parameters
void DrawAllBuildingEavesAndFootprint(vector<Building3D*> vec_of_buildings, GLfloat* color, int pointsize) {
	glPointSize(pointsize);
	glColor3fv(color); // set drawing color
	for (Building3D* building : vec_of_buildings) {
		DrawPolygonOnSpecifiedHeight(building->polygon, height_factor * building->eaves_height); // draw edges of easvesline
		DrawPolygonOnSpecifiedHeight(building->polygon, height_factor * building->foot_height); // draw edges of footprint
	}
}

// draws vertical edges of specified height as corners of specified vector of buildings
void DrawAllBuildingVerticalEdges(vector<Building3D*> vec_of_buildings, GLfloat* color, int pointsize) {
	glPointSize(pointsize);
	glColor3fv(color); // set drawing color
	for (Building3D* building : vec_of_buildings) {
		glBegin(GL_LINES); // draw outer contour
		for (Point3 p : building->polygon->outer_contour.points) {
			glVertex3f(p.x, p.y, height_factor * building->eaves_height);
			glVertex3f(p.x, p.y, height_factor * building->foot_height);
		}
		glEnd();

		for (Contour3D c : building->polygon->inner_contours) { // draw each inner contour
			glBegin(GL_LINES);
			for (Point3 p : c.points) {
				glVertex3f(p.x, p.y, height_factor * building->eaves_height);
				glVertex3f(p.x, p.y, height_factor * building->foot_height);
			}
			glEnd();
		}
	}
}

// draws vertical rectangles of specified height as walls of specified vector of buildings
void DrawAllPolygonVerticalPlanes(vector<Building3D*> vec_of_buildings, GLfloat* color, int pointsize) {
	Point3 p1, p2;
	glPointSize(pointsize);
	glColor3fv(color); // set drawing color
	for (Building3D* building : vec_of_buildings) {
		glBegin(GL_QUADS); // draw outer contour
		for (int i = 0; i < building->polygon->outer_contour.points.size() - 1; i++) {
			p1 = building->polygon->outer_contour.points[i];
			p2 = building->polygon->outer_contour.points[i + 1];
			glVertex3f(p1.x, p1.y, height_factor * building->eaves_height);
			glVertex3f(p2.x, p2.y, height_factor * building->eaves_height);
			glVertex3f(p2.x, p2.y, height_factor * building->foot_height);
			glVertex3f(p1.x, p1.y, height_factor * building->foot_height);
		}
		glEnd();

		for (Contour3D c : building->polygon->inner_contours) { // draw each inner contour
			glBegin(GL_QUADS);
			for (int i = 0; i < c.points.size() - 1; i++) {
				p1 = c.points[i];
				p2 = c.points[i + 1];
				glVertex3f(p1.x, p1.y, height_factor * building->eaves_height);
				glVertex3f(p2.x, p2.y, height_factor * building->eaves_height);
				glVertex3f(p2.x, p2.y, height_factor * building->foot_height);
				glVertex3f(p1.x, p1.y, height_factor * building->foot_height);
			}
			glEnd();
		}
	}
}

// draws the scene
void DisplayMe(void) {
	glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
	glClear(GL_COLOR_BUFFER_BIT);

	// find median and range of point cloud
	vector<Point3> medran = Median_and_range_of_vector(points_dsm);
	float multiple;
	if (medran[1].x > medran[1].y) // define multiple with respect to biggest dimension of pointcloud
		multiple = max_coord / medran[1].x;
	else
		multiple = max_coord / medran[1].y;

	glPushMatrix();
		glTranslatef(shift_x, shift_y, 0.0f); // translation (shift_x, shift_y)
		glRotatef(theta_x, 0, 1, 0); // rotation (angle theta_x around axis Y)
		glRotatef(theta_y, 1, 0, 0); // rotation (angle theta_y around axis X)
		glScalef(scale, scale, scale); // scale using scale value

		glTranslatef(-multiple * medran[0].x, -multiple * medran[0].y, 0.0f);
		glScalef(multiple, multiple, 1.0); // scale using multiple value

		if (on_0) { // draws boxes of buildings
			DrawAllPolygonVerticalPlanes(vec_of_buildings, WHEAT, 1); // draw wheat-colored walls

			for (Building3D* building : vec_of_buildings) {  // for every member of vec_of_flatroofs
				tesselationOfPolygonWithHoles(building->polygon); // create tesselated poly
				DrawTesselated(height_factor * building->eaves_height, BROWN); // draw brown roofs
			}
		}

		if (on_9) { // draws vertical edges and edges of eaves and footprints 
			DrawAllBuildingEavesAndFootprint(vec_of_buildings, BLUE, 1); // draws footprint and eavesline of every building
			DrawAllBuildingVerticalEdges(vec_of_buildings, BLUE, 1); // draws vertical edges of every building
		}

		if (on_1) DrawAllPointFeatures(points_dsm_copy, BLACK, 1); // draws all points of DSM
		if (on_2) DrawAllPointFeatures(points_dtm, DARK_GREEN, 1); // draws all points of DTM
		if (on_3) DrawAllPointFeatures(points_above_ground_copy, DARK_CYAN, 2); // draws all points which are above the ground
		if (on_4) DrawAllPointFeatures(points_ground, BROWN, 2); // draws all points which represent ground
		if (on_5) DrawAllPointFeatures(points_veget, GREEN, 2); // draws all points which represent vegetation

		if (on_6) // draws all clusters of points
			for (int i = 0; i < cluster_array.size(); i++) DrawAllPointFeatures(cluster_array[i], DARK_YELLOW, 2); 

		if (on_7) // draws all points of outlines
			for (int i = 0; i < outl_array.size(); i++) DrawAllPointFeatures(outl_array[i], DARK_YELLOW, 2); 
			
		if (on_8) // draws all points of douglas-peuckers
			for (int i = 0; i < dgp_array.size(); i++) DrawAllPointFeatures(dgp_array[i], DARK_YELLOW, 2); 
		
	glPopMatrix();
	glFlush();
}

void ChangeSize(GLsizei width, GLsizei height) {
	if (height == 0) height = 1;
	window_pix_width = (float)width;
	window_pix_height = (float)height;

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity(); // initialized to identity matrix
	GLfloat AspectRatio = (GLfloat)width / (GLfloat)height; // window's aspect ratio
	GLfloat deep = 200;
	if (width <= height) glOrtho(-1.0 * max_coord, 1.0 * max_coord, -1.0 / AspectRatio * max_coord, 1.0 / AspectRatio * max_coord, deep, -1 * deep);
	else glOrtho(-1.0 * AspectRatio * max_coord, 1.0 * AspectRatio * max_coord, -1.0 * max_coord, 1.0 * max_coord, deep, -1 * deep);

	glViewport(0, 0, width, height); // size of viewport (in pixels), now viewport covers the entire new resized window
}

// function defines reaction on keyboard activity
void KeyReaction(unsigned char key, int x, int y) {
	switch (key) {
	case '1':
		on_1 = !on_1;
		break;
	case '2':
		on_2 = !on_2;
		break;
	case '3':
		on_3 = !on_3;
		break;
	case '4':
		on_4 = !on_4;
		break;
	case '5':
		on_5 = !on_5;
		break;
	case '6':
		on_6 = !on_6;
		break;
	case '7':
		on_7 = !on_7;
		break;
	case '8':
		on_8 = !on_8;
		break;
	case '9':
		on_9 = !on_9;
		break;
	case '0':
		on_0 = !on_0;
		break;
	}

	DisplayMe();
}

// function defines reaction on mouse buttons activity
void mouse_button(int button, int state, int x, int y)
{
	mouse_button_event_pix_x = x;
	mouse_button_event_pix_y = y;
	if (button == GLUT_LEFT_BUTTON && state == GLUT_DOWN)
		left_button = true;
	if (button == GLUT_LEFT_BUTTON && state == GLUT_UP)
		left_button = false;
	if (button == GLUT_RIGHT_BUTTON && state == GLUT_DOWN)
		right_button = true;
	if (button == GLUT_RIGHT_BUTTON && state == GLUT_UP)
		right_button = false;
	if (button == GLUT_MIDDLE_BUTTON && state == GLUT_DOWN)
		middle_button = true;
	if (button == GLUT_MIDDLE_BUTTON && state == GLUT_UP)
		middle_button = false;
}

// function defines reaction on mouse motion
void mouse_motion(int mouse_motion_coor_pix_x, int mouse_motion_coor_pix_y)
{

	float dx = (float)(mouse_motion_coor_pix_x - mouse_button_event_pix_x) / window_pix_width;
	float dy = (float)(mouse_motion_coor_pix_y - mouse_button_event_pix_y) / window_pix_height;

	if (right_button) {
		shift_x += dx * 80;
		shift_y -= dy * 80;
	}

	if (left_button) {
		theta_x += dx * 360.0;
		theta_y += dy * 360.0;
	}

	if (middle_button) {
		scale += dy * 7;
	}

	mouse_button_event_pix_x = mouse_motion_coor_pix_x;
	mouse_button_event_pix_y = mouse_motion_coor_pix_y;

	DisplayMe();
}

int main(int argc, char** argv)
{
	Sand_Ex1 sandex1;
	// read initial pointcloud
	cout << "Lets read point file." << endl;
	points_dsm = TXTReader::ReadFileToVectorOfPointers("scanning_results.xyz");
	cout << points_dsm.size() << " points read." << endl;

	if (points_dsm.size() > 0) {
		// define the minimal virtual point (offset of initial pointcloud) and move points to the origin
		const Point3 OFFSET = sandex1.virt_min_xyz(points_dsm);
		sandex1.move_points(points_dsm, OFFSET); // now points_dsm itself is near (0,0,0)
		points_dsm_copy = sandex1.clone(points_dsm);

		// transform DSM to DTM
		points_dtm = sandex1.build_DTM_from_DSM(points_dsm, 49.5, 0.5, 3.0);

		// Determine which points are above ground by points_above_DTM function
		vector<Point3*> points_above_ground = sandex1.points_above_DTM(points_dsm, points_dtm, points_ground, 4, 0);
		points_above_ground_copy = sandex1.clone(points_above_ground);
		cout << "Point cloud above ground has size " << points_above_ground.size() << endl;

		// Deleting of low-density points was used before development of clusterisation
		vector<Point3*> buildings_dense = sandex1.density_threshold(points_above_ground, 1.0, 3);
		cout << "Dense part of the cloud is " << buildings_dense.size() << " points." << endl;

		// create neighborhood relations in the cloud
		cout << "Determination of neighborhood relations" << endl;
		sandex1.determine_neighborhood(buildings_dense, 1.0);

		cout << "Clusterisation of point cloud" << endl;
		cluster_array = sandex1.clusterize_point_cloud_to_vec_vec_pointers(buildings_dense);

		cout << "Delete too small clusters" << endl;
		sandex1.delete_small_clusters(cluster_array, 100);
		cout << "Separate vegetation from buildings" << cluster_array.size() << endl;

		sandex1.separate_vegetation(cluster_array, points_veget, 0.1);
		cout << "Cluster of buildings has size " << cluster_array.size() << endl;

		vec_of_buildings = sandex1.vector_of_flatroofs_from_clusterised_point_cloud(cluster_array, outl_array, dgp_array);

		// foot_height of every building is found from calculation of minimal value of "points_dtm[s]->z" from dougpeu of current building
		for (int i = 0; i < vec_of_buildings.size(); i++) { // for every building
			for (int j = 0; j < dgp_array[i].size(); j++) { // for every point of dougpeu
				for (int s = 0; s < points_dsm.size(); s++) { // for every point of points_dsm
					if (points_dsm[s] == dgp_array[i][j]) { // if this point of dougpeu corresponds to this point of dsm
						// statement tries to decrease foot_height of current building
						if (points_dtm[s]->z < vec_of_buildings[i]->foot_height) {// if this point of dsm is lower than current value of foot_height for this building
							vec_of_buildings[i]->foot_height = points_dtm[s]->z; // reassign value of foot_height
						}
					}
				}
			}
		}

		// reduce heights
		float height_diff = sandex1.reduce_heights(vec_of_buildings);
		sandex1.reduce_cloud_heights(points_dsm_copy, height_diff);
		sandex1.reduce_cloud_heights(points_dtm, height_diff);
		sandex1.reduce_cloud_heights(points_above_ground_copy, height_diff);
		sandex1.reduce_cloud_heights(points_ground, height_diff);
		sandex1.reduce_cloud_heights(points_veget, height_diff);
		for (int i = 0; i < cluster_array.size(); i++) sandex1.reduce_cloud_heights(cluster_array[i], height_diff);

		// OpenGl part

		glutInit(&argc, argv); // starts window system management session
		glutInitDisplayMode(GLUT_SINGLE); // display mode setting (option - rgb)
		glutInitWindowSize(1400, 800); // define size
		glutInitWindowPosition(0, 0); // and position of the window
		glutCreateWindow("Default city:"); // define title of the window

		glutDisplayFunc(DisplayMe);
		glutReshapeFunc(ChangeSize);
		glutKeyboardFunc(KeyReaction);
		glutMouseFunc(mouse_button);
		glutMotionFunc(mouse_motion);
		glutMainLoop(); // starts listening for events
	}

	system("PAUSE");
	return 0;
}