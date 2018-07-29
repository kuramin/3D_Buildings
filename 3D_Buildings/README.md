<html>
<body>
	<h2>3D building reconstruction and visualisation</h2>
	<p>Current project performs classification of a point cloud to Ground, Vegetation and Buildings and than visualizes members of these classes using OpenGL library. The input point cloud is a part of result of aerial laser scanning of a village. The input point cloud looks like the following:</p>
	<img src="images/bui1.png" width="99%"> <br> <br>
	<p>Result of classification is presented on the next picture. Brown points represent Ground, green points are Vegetation, beige points belong to Buildings:</p>
	<img src="images/bui2.png" width="1000" height="350"> <br> <br>
	<p>During execution the program performs the following actions:<ol class="ord_list" type="a">
		<li>Read point cloud from .xyz file;</li>
		<li>Transfer point cloud to the local coordinate system;</li>
		<li>Treat point cloud as a Digital Surface Model in order to determine Digital Terrain Model using filtering technique;</li>
		<li>Separate points of DSM which have height difference with DTM more than threshold from those points which are considered being ground;</li>
		<li>Take only those parts of the resulting point cloud which are dense regions;</li>
		<li>Determine neighborhood relations in the resulting point cloud;</li>
		<li>Clusterize the point cloud and delete too small clusters;</li>
		<li>Use approach similar to RANSAC to separate clusters of buildings from clusters of vegetation;</li>
		<li>For every cluster of a building define its outline by Modified Convex Hull method;</li>
		<li>Simplify every building outline by Douglas-Peucker;</li>
		<li>Create a set of building models on level of details LoD1 based on building simplified outlines. 
			Height of the ground floor is assigned based on the DTM. 
			Height of the roof is calculated by averaging of all heights of current cluster members;</li>
		<li>Visualize intermediate and final results.</li>
	</ol>
	</p> <br> <br>
	<p>Interface description:
	<ol class="ord_list" type="a">
	<li>In the beginning program shows comments about performed steps, shows number of read points, number of clusters and etc;</li>
	<li>In the window "Default city" the visualization appears, initially only the input point cloud is shown;</li>
	<li>Visualisation can be rotated by left mouse button, translated by right mouse button and scaled by pushing mouse scroll together with mouse motion;</li>
	<li>Pushing of the following buttons switches ON and OFF the corresponding results:<ol>
		<li> - input point cloud (DSM);</li>
		<li> - Digital Terrain Model (DTM);</li>
		<li> - points which are above ground;</li>
		<li> - points which represent ground;</li>
		<li> - points which are above ground and represent vegetation;</li>
		<li> - points which are above ground and represent buildings;</li>
		<li> - points of outlines of buildings (calculated by Modified Convex Hull);</li>
		<li> - points of simplified outlines (calculated by Douglas-Peucker);</li>
		<li> - edges of created building models;</li>
		<li> - building models; Beige color represents walls, brown color - roofs.</li> </ol> </li>
	<li>For exit push the Close button.</li>
	</ol>
	</p> <br> <br>
	<p>The following image shows vegetation and building outlines only:</p>
	<img src="images/bui3.png" width="1000" height="350"> <br> <br>
	<p>Visualization of vegetation and building models:</p>
	<img src="images/bui4.png" width="1000" height="350">
</body>
</html>