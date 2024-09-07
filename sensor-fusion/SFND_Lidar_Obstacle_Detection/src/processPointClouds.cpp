// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"

//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}

// # define STANDARD_CLUTERING_LIBRARY
//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    // std::cout << cloud->points.size() << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint) {
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
    typename pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT> ()); 
    typename pcl::PointCloud<PointT>::Ptr cloud_region_cropped (new pcl::PointCloud<PointT> ()); 
    typename pcl::PointCloud<PointT>::Ptr cloud_roof_cropped (new pcl::PointCloud<PointT> ()); 

    pcl::VoxelGrid<PointT> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(filterRes, filterRes, filterRes);
    sor.filter(*cloud_filtered);

    pcl::CropBox<PointT> region(true);
    region.setMin(minPoint);
    region.setMax(maxPoint);
    region.setInputCloud(cloud_filtered);
    region.filter(*cloud_region_cropped);

    std::vector<int> indices;

    pcl::CropBox<PointT> roof(true);
    roof.setMin(Eigen::Vector4f(-1.5, -1.7, -1, 1));
    roof.setMax(Eigen::Vector4f(2.6, 1.7, -0.4, 1));
    roof.setInputCloud(cloud_region_cropped);
    roof.filter(indices);

    pcl::PointIndices::Ptr inliers {new pcl::PointIndices};

    for (int point: indices) {
        inliers->indices.push_back(point);
    }

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud_region_cropped);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloud_roof_cropped);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloud_roof_cropped;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
    typename pcl::PointCloud<PointT>::Ptr obstacle_cloud (new pcl::PointCloud<PointT> ()); 
    typename pcl::PointCloud<PointT>::Ptr plane_cloud (new pcl::PointCloud<PointT> ()); 

    for (int index: inliers->indices) {
        plane_cloud->points.push_back(cloud->points[index]);
    }

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*obstacle_cloud);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstacle_cloud, plane_cloud);
    return segResult;
}

template<typename PointT>
pcl::PointIndices::Ptr ProcessPointClouds<PointT>::RansacP3D(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{
    pcl::PointIndices::Ptr inliersResult (new pcl::PointIndices ());
	srand(time(NULL));
	
    if (cloud->points.size() == 0) {
        std::cout<<"The cloud points size was 0."<<std::endl;
        return inliersResult;
    }

	// std::cout<<"Ransac (point cloud): "<<cloud->size()<<std::endl;
	
    for (int i = 0; i < maxIterations; i++) {
		int first_point = rand() % cloud->size();
		int second_point = first_point;
		int third_point = first_point;
		
		while (second_point == first_point) {
			second_point = rand() % cloud->size();
		}

		while (second_point == first_point || third_point == first_point) {
			third_point = rand() % cloud->size();
		}

		// std::cout<<"Iteration : "<<i<<" "<<typeid(cloud->points[first_point]).name()<<"points = "<<first_point<<" "<<second_point<<" "<<third_point<<std::endl;
		PointT P1 = cloud->points[first_point];
		PointT P2 = cloud->points[second_point];
		PointT P3 = cloud->points[third_point];
		
		double A1 = P2.x - P1.x;
		double B1 = P2.y - P1.y;
		double C1 = P2.z - P1.z;
		
		double A2 = P3.x - P1.x;
		double B2 = P3.y - P1.y;
		double C2 = P3.z - P1.z; 

		double A = B1*C2-C1*B2; 
		double B = C1*A2-A1*C2; 
		double C = A1*B2-B1*A2;

		double D = -1 * (A*P1.x + B*P1.y + C*P1.z); 

        pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
		for (int index = 0; index < cloud->points.size(); index++) {
			PointT point = cloud->points[index];
			double distance = abs((point.x*A)+(point.y*B)+(point.z*C)+D)/sqrt(A*A + B*B + C*C);
			if (distance <= distanceTol) {
				inliers->indices.push_back(index);
			}
		}

		if (inliers->indices.size() > inliersResult->indices.size()) {
			inliersResult = inliers;
		}
	}
	
	return inliersResult;
}
template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
    pcl::PointIndices::Ptr inliers;

    #ifdef SEGMENTATION_RANSAC_STDLIB
	pcl::PointIndices::Ptr inliers {new pcl::PointIndices};  

    // TODO:: Fill in this function to find inliers for the cloud.
    pcl::ModelCoefficients::Ptr coefficients {new pcl::ModelCoefficients};
    pcl::SACSegmentation<PointT> seg;

    seg.setOptimizeCoefficients (true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);

    seg.setMaxIterations(maxIterations);
    seg.setDistanceThreshold(distanceThreshold);

    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);

    if (inliers->indices.size() == 0) {
        std::cout<<"Could not estimate the planar model for the dataset provided."<<std::endl;
    }
    #else 
    inliers = RansacP3D(cloud, maxIterations, distanceThreshold);
    #endif 

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}

template<typename PointT>
void ProcessPointClouds<PointT>::proximity(int idx, 
	typename pcl::PointCloud<PointT>* cluster, 
	std::unordered_map<int, bool>* visited, 
	KdTree* tree, 
	typename pcl::PointCloud<PointT>::Ptr cloud, 
    float distanceTol) {
	if (!(*visited)[idx]) {		
        std::vector<double>* vec = new std::vector<double>{cloud->points[idx].x, cloud->points[idx].y, cloud->points[idx].z};
		std::vector<int> nearby = tree->search(*vec, distanceTol);
		(*visited)[idx] = true; 

		for(int index : nearby) {
            // std::cout<<"Index = "<<index<<" "<<cloud->points.size()<<*cluster<<std::endl;
            if (!(*visited)[index]) {	
                cluster->push_back(cloud->points[index]);
                // std::cout<<"Added point to the cluster = "<<index<<std::endl;
			    proximity(index, cluster, visited, tree, cloud, distanceTol);
            }
		}
	} else {
        std::cout<<"Already visited"<<std::endl;
    }
}

template<typename PointT> 
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::euclideanCluster(typename pcl::PointCloud<PointT>::Ptr cloud, KdTree* tree, float distanceTol, int minSize, int maxSize)
{
    std::cout<<"Euclidean cluster "<<cloud->points.size()<<std::endl;
	std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;
	std::unordered_map<int, bool>* visited = new std::unordered_map<int, bool>();

	for (int idx = 0; idx < cloud->points.size(); idx++) {
		if (!(*visited)[idx]) {
			typename pcl::PointCloud<PointT> cluster; 			

			proximity(idx, &cluster, visited, tree, cloud, distanceTol);

			if (cluster.points.size() >= minSize && cluster.points.size() <= maxSize ) {
                clusters.push_back(cluster.makeShared());
            }
		}
	}
 
	return clusters;
}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{
    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();
    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    #ifdef STANDARD_CLUTERING_LIBRARY
        typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);

        tree->setInputCloud (cloud);

        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<PointT> ec;
        ec.setClusterTolerance (clusterTolerance); 
        ec.setMinClusterSize (minSize);
        ec.setMaxClusterSize (maxSize);
        ec.setSearchMethod (tree);
        ec.setInputCloud (cloud);
        ec.extract (cluster_indices);
    #else 
    	KdTree* tree = new KdTree();

        for (int i=0; i<cloud->points.size(); i++) {
            std::vector<double>* vec = new std::vector<double>{cloud->points[i].x, cloud->points[i].y, cloud->points[i].z};
    	    tree->insert(*vec,i); 
        }

        clusters = euclideanCluster(cloud, tree, clusterTolerance, minSize, maxSize);
    #endif 


    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout <<cloud->points.size()<< "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}


template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}