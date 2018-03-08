Intro
Current project performs classification of a point cloud to classes of Ground, 
Vegetation and Buildings and than visualizes members of these classes using OpenGL. 
The input point cloud is a part of result of aerial laser scanning of a village. 
The exe-file "3D_Buildings_prj.exe" is located in the folder "../3D_Buildings/3D_Buildings/3D_Buildings_prj/Release/

Description
During execution the program performs the following actions:
1) Read point cloud from .xyz file;
2) Transfer point cloud to the local coordinate system;
3) Treat point cloud as a Digital Surface Model in order to determine Digital Terrain Model using filtering technique;
4) Separate points of DSM which have height difference with DTM more than threshold from those points which are considered being ground;
5) Take only those parts of the resulting point cloud which are dense regions;
6) Determine neighborhood relations in the resulting point cloud;
7) Clusterize the point cloud and delete too small clusters;
8) Use approach similar to RANSAC to separate clusters of buildings from clusters of vegetation;
9) For every cluster of a building define its outline by Modified Convex Hull method;
10) Simplify every building outline by Douglas-Peucker;
11) Creates a set of building models on level of details LoD1 based on building simplified outlines. 
    Height of the ground floor is assigned based on the DTM. 
    Height of the roof is calculated by averaging of all heights of current cluster members;
12) Visualizes intermediate and final results.

Visualization
1) In the beginning program shows comments about performed steps, shows number of read points, number of clusters and etc;
2) In the window "Default city" the visualization appears, initially only the input point cloud is shown;
3) Visualisation can be rotated by left mouse button, translated by right mouse button and scaled by pushing mouse scroll together with mouse motion;
4) Pushing of the following buttons switches ON and OFF the corresponding results:
	1 - input point cloud (DSM);
	2 - Digital Terrain Model (DTM);
	3 - points which are above ground;
	4 - points which represent ground;
	5 - points which are above ground and represent vegetation;
	6 - points which are above ground and represent buildings;
	7 - points of outlines of buildings (calculated by Modified Convex Hull);
	8 - points of simplified outlines (calculated by Douglas-Peucker);
	9 - edges of created building models;
	10 - building models; Beige color represents walls, brown color - roofs.
5) For exit push the Close button.
