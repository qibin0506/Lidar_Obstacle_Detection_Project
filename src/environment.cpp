/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

const bool USE_PCL = false;

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

void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI> pointProcess, pcl::PointCloud<pcl::PointXYZI>::Ptr clouds) {
// void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer) {
//   ProcessPointClouds<pcl::PointXYZI>* pointProcess = new ProcessPointClouds<pcl::PointXYZI>();
//   pcl::PointCloud<pcl::PointXYZI>::Ptr clouds = pointProcess->loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");
  
  ProcessPointClouds<pcl::PointXYZI> pointProcessor;
  clouds = pointProcessor.FilterCloud(clouds, 0.3, Eigen::Vector4f(-10, -5, -2, 1), Eigen::Vector4f(30, 8, 1, 1));
  
  std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segResult;
  if (USE_PCL) {
    segResult = pointProcessor.SegmentPlane(clouds, 25, 0.3);
  } else {
    segResult = pointProcessor.Ransac(clouds, 100, 0.2);
  }

  renderPointCloud(viewer, segResult.second, "planeCloud", Color(0, 1, 0));
  
  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters;
  if(USE_PCL) {
    cloudClusters = pointProcessor.Clustering(segResult.first, 0.53, 10, 500);
  } else {
    KdTree* tree = new KdTree;
    for (int i = 0; i < segResult.first->points.size(); i++) {
     tree->insert(segResult.first->points[i], i);
    }

    cloudClusters = pointProcessor.EuclideanClustering(segResult.first, tree, 0.53, 30, 250);
  }
  
  int clusterId = 0;
  std::vector<Color> colors = {Color(1, 0, 0), Color(1, 1, 0), Color(0, 0, 1)};
  
  for (pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters) {
    pointProcessor.numPoints(cluster);
    renderPointCloud(viewer, cluster, "obstCloud" + std::to_string(clusterId), colors[clusterId%colors.size()]);
    
    Box box = pointProcessor.BoundingBox(cluster);
    renderBox(viewer, box, clusterId);
    clusterId++;
  }
  
//   renderPointCloud(viewer, clouds, "inputCloud");
}

void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
  // ----------------------------------------------------
  // -----Open 3D viewer and display simple highway -----
  // ----------------------------------------------------
  
  // RENDER OPTIONS
  bool renderScene = true;
  std::vector<Car> cars = initHighway(renderScene, viewer);
  
  // TODO:: Create lidar sensor 
  Lidar* lidar = new Lidar(cars, 0);
  pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud = lidar->scan();
  // renderRays(viewer, lidar->position, inputCloud);
  renderPointCloud(viewer, inputCloud, "inputCloud");

  // TODO:: Create point processor
  ProcessPointClouds<pcl::PointXYZ> pointProcessor;
  std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segResult = pointProcessor.SegmentPlane(inputCloud, 100, 0.2);

  renderPointCloud(viewer, segResult.first, "obstCloud", Color(1, 0, 0));
  renderPointCloud(viewer, segResult.second, "planeCloud", Color(0, 1, 0));
  
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = pointProcessor.Clustering(segResult.first, 1.0, 3, 30);
  
  int clusterId = 0;
  std::vector<Color> colors = {Color(1, 0, 0), Color(1, 1, 0), Color(0, 0, 1)};
  
  for (pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters) {
    std::cout << "hah" << clusterId << std::endl;
    pointProcessor.numPoints(cluster);
    renderPointCloud(viewer, cluster, "obstCloud" + std::to_string(clusterId), colors[clusterId%colors.size()]);
    
    Box box = pointProcessor.BoundingBox(cluster);
    renderBox(viewer, box, clusterId);
    clusterId++;
  }
}


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
//     simpleHighway(viewer);
//   cityBlock(viewer);
  
   ProcessPointClouds<pcl::PointXYZI> pointProcess;
  std::vector<boost::filesystem::path> stream = pointProcess.streamPcd("../src/sensors/data/pcd/data_1");
  auto streamIterator = stream.begin();
  pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;

    while (!viewer->wasStopped ())
    {
      	viewer->removeAllPointClouds();
      	viewer->removeAllShapes();
      	
      	inputCloudI = pointProcess.loadPcd((*streamIterator).string());
      	cityBlock(viewer, pointProcess, inputCloudI);
      	streamIterator++;
      	if (streamIterator >= stream.end()) {
          streamIterator = stream.begin();
        }
      	
        viewer->spinOnce ();
    } 
}
