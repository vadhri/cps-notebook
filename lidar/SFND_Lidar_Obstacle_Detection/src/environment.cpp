/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"
#define STREAMING_PCD
std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    Car egoCar( Vect3(0,0,0), Vect3(4,2,2), Color(0,1,0), "egoCar");
    Car car1( Vect3(15,0,0), Vect3(4,2,2), Color(0,0,1), "car1");
    Car car2( Vect3(8,-4,0), Vect3(4,2,2), Color(0,0,1), "car2");	
    Car car3( Vect3(-12,4,0), Vect3(4,2,2), Color(0,0,1), "car3");
  
    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if(renderScene)
    {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}

#ifdef STREAMING_PCD
void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, 
    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI, 
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud)
#else
void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer)
#endif
{
  // ----------------------------------------------------
  // -----Open 3D viewer and display City Block     -----
  // ----------------------------------------------------
  
#ifndef STREAMING_PCD
    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud = pointProcessorI->loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");
    //   renderPointCloud(viewer,inputCloud,"inputCloud");
#endif

    if (inputCloud == NULL) {
        std::cout<<"inputCloud = "<<inputCloud<<std::endl;
        return;
    }
    
    std::cout<<"The cloud size is = "<<inputCloud->points.size()<<std::endl;

    if (inputCloud->points.size() == 0) {
        return;
    }


    pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud;
    filterCloud = pointProcessorI->FilterCloud(inputCloud, 0.35f, Eigen::Vector4f (-10, -7, -2, 1), Eigen::Vector4f (30, 7, 1, 1));
    // renderPointCloud(viewer,filterCloud,"filterCloud");

    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmented_cloud = pointProcessorI->SegmentPlane(filterCloud, 40, 0.3);
    // renderPointCloud(viewer, segmented_cloud.first, "obstacleCloud", Color(1,0,0));
    renderPointCloud(viewer, segmented_cloud.second, "Road", Color(0,1,0));

    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI->Clustering(segmented_cloud.first, 1.0, 10, 1400);
        
    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1),Color(1,0,1), Color(1,1,0),Color(0,1,1), Color(0,0.75,0.75)};
    
    for(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters)
    {
        // std::cout << "cluster size ";
        pointProcessorI->numPoints(cluster);
        renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId]);
        Box box = pointProcessorI->BoundingBox(cluster);
        renderBox(viewer,box,clusterId);
        ++clusterId;
    }        
}


// void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
// { 
//     // ----------------------------------------------------
//     // -----Open 3D viewer and display simple highway -----
//     // ----------------------------------------------------
    
//     // RENDER OPTIONS
//     bool renderScene = false;
//     bool render_point_cloud = false;
//     std::vector<Car> cars = initHighway(renderScene, viewer);
//     int8_t slope = 0;

//     // TODO:: Create lidar sensor 
//     Lidar* lidar = new Lidar(cars, slope);
//     pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud = lidar->scan();

//     if (renderScene) {
//         renderRays(viewer, lidar->position, point_cloud);
//     } else if (render_point_cloud) {
//         renderPointCloud(viewer, point_cloud, "point_cloud", Color(0,0,1));
//     }
    
//     // TODO:: Create point processor
//     ProcessPointClouds<pcl::PointXYZ> *pointProcessor = new ProcessPointClouds<pcl::PointXYZ>();
//     std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmented_cloud = pointProcessor->SegmentPlane(point_cloud, 100, 0.02);
//     // renderPointCloud(viewer, segmented_cloud.first, "obstacleCloud", Color(1,0,0));
//     renderPointCloud(viewer, segmented_cloud.second, "planeCloud", Color(0,1,0));

//     std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = pointProcessor->Clustering(segmented_cloud.first, 1.0, 3, 30);
        
//     int clusterId = 0;
//     std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1),Color(1,0,1), Color(1,1,0),Color(0,1,1), Color(0,0.75,0.75)};
    
//     for(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters)
//     {
//         // std::cout << "cluster size ";
//         pointProcessor->numPoints(cluster);
//         renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId]);
//         Box box = pointProcessor->BoundingBox(cluster);
//         renderBox(viewer,box,clusterId);
//         ++clusterId;
//     }    
// }   


//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    viewer->setBackgroundColor (0, 0, 0);
    
    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;
    
    switch(setAngle)
    {
        case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if(setAngle!=FPS)
        viewer->addCoordinateSystem (1.0);
}

int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);
    // simpleHighway(viewer);
    #ifdef STREAMING_PCD
        ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
        std::vector<std::filesystem::path> stream = pointProcessorI->streamPcd("../src/sensors/data/pcd/data_1");
        // std::vector<std::filesystem::path> stream = pointProcessorI->streamPcd("../src/sensors/data/pcd/data_2");
        auto streamIterator = stream.begin();
        pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;
        cityBlock(viewer, pointProcessorI, inputCloudI);
    #else
        cityBlock(viewer);
    #endif

    while (!viewer->wasStopped ())
    {
        #ifdef STREAMING_PCD
            // Clear viewer
            viewer->removeAllPointClouds();
            viewer->removeAllShapes();

            // Load pcd and run obstacle detection process
            inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
            cityBlock(viewer, pointProcessorI, inputCloudI);
                
            streamIterator++;
            if(streamIterator == stream.end())
                streamIterator = stream.begin();
        #endif 

        viewer->spinOnce ();
    } 
}