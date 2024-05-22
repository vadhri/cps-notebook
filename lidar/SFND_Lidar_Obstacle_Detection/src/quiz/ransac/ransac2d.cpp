/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  	// Add inliers
  	float scatter = 0.6;
  	for(int i = -5; i < 5; i++)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = i+scatter*rx;
  		point.y = i+scatter*ry;
  		point.z = 0;

  		cloud->points.push_back(point);
  	}
  	// Add outliers
  	int numOutliers = 10;
  	while(numOutliers--)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = 5*rx;
  		point.y = 5*ry;
  		point.z = 0;

  		cloud->points.push_back(point);

  	}
  	cloud->width = cloud->points.size();
  	cloud->height = 1;

  	return cloud;

}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D()
{
	ProcessPointClouds<pcl::PointXYZ> pointProcessor;
	return pointProcessor.loadPcd("../../../src/sensors/data/pcd/simpleHighway.pcd");
}


pcl::visualization::PCLVisualizer::Ptr initScene()
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
  	viewer->initCameraParameters();
  	viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
  	viewer->addCoordinateSystem (1.0);
  	return viewer;
}

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// TODO: Fill in this function
	std::cout<<"Ransac (point cloud): "<<cloud->size()<<std::endl;
	// For max iterations 
	for (int i = 0; i < maxIterations; i++) {
		int first_point = rand() % cloud->size();
		int second_point = first_point;
		int third_point = first_point;
		
		while (second_point == first_point) {
			second_point = rand() % cloud->size();
		}

		while (second_point == first_point && third_point == first_point) {
			third_point = rand() % cloud->size();
		}

		std::cout<<"Iteration : "<<i<<" "<<typeid(cloud->points[first_point]).name()<<"points = "<<first_point<<" "<<second_point<<" "<<third_point<<std::endl;
		pcl::PointXYZ P1 = cloud->points[first_point];
		pcl::PointXYZ P2 = cloud->points[second_point];
		pcl::PointXYZ P3 = cloud->points[third_point];
		
		double A1 = P2.x - P1.x;
		double B1 = P2.y - P1.y;
		double C1 = P2.z - P1.z;
		
		double A2 = P3.x - P1.x;
		double B2 = P3.y - P1.y;
		double C2 = P3.z - P1.z; 

		double A = A1 * A2; 
		double B = B1 * B2; 
		double C = C1 * C2;

		double D = -1 * (A*P1.x + B*P1.y + C*P1.z); 
		
		std::unordered_set<int> inliers;
		for (int index = 0; index < cloud->points.size(); index++) {
			pcl::PointXYZ point = cloud->points[index];
			double distance = abs(point.x*A+point.y*B+point.z*C+D)/sqrt(A*A + B*B + C*C);
			if (distance <= distanceTol) {
				inliers.insert(index);
			}
		}

		if (inliers.size() > inliersResult.size()) {
			inliersResult = inliers;
		}
	}
	
	return inliersResult;
}

int main ()
{
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();
	std::unordered_set<int> inliers = Ransac(cloud, 10, 1.0);

	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZ point = cloud->points[index];
		if(inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}

	if(inliers.size()) {
		renderPointCloud(viewer,cloudInliers,"inliers",Color(0,1,0));
  		renderPointCloud(viewer,cloudOutliers,"outliers",Color(1,0,0));
	}
  	else
  	{
  		renderPointCloud(viewer,cloud,"data");
  	}
	
  	while (!viewer->wasStopped ())
  	{
  	  viewer->spinOnce ();
  	}
  	
}
