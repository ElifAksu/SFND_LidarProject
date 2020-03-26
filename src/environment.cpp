/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"
#include "ransac.h"

#include "cluster_pcd.h"

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


void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
    bool renderScene = false;
    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    // TODO:: Create lidar sensor 
    Lidar *lidar =new Lidar(cars,0);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = lidar->scan();
    //renderRays(viewer, lidar->position, cloud);
    //renderPointCloud(viewer, cloud,"PCD");
        // TODO:: Create point processor
   
     ProcessPointClouds<pcl::PointXYZ> processor;
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr,pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud=processor.SegmentPlane(cloud, 100,0.2);
    
    renderPointCloud(viewer, segmentCloud.second,"plane",Color(0,1,0));
    renderPointCloud(viewer, segmentCloud.first,"obstacle",Color(1,0,0));
    


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
void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>* processorI, const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud)

{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    //pcl::PointCloud<pcl::PointXYZI>::Ptr inputcloud = processorI.loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered =processorI->FilterCloud(inputCloud,0.3, Eigen::Vector4f(-10,-5,-2,1), Eigen::Vector4f(30,7,1,1 ) );
    renderPointCloud(viewer,inputCloud,"PCD");
    std::unordered_set<int> inliers = Ransac3D_func(cloud_filtered, 500,0.2);

    pcl::PointCloud<pcl::PointXYZI>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr NonGround(new pcl::PointCloud<pcl::PointXYZI>());

    for(int index = 0; index < cloud_filtered->points.size(); index++)
    {
        pcl::PointXYZI point = cloud_filtered->points[index];
        if(inliers.count(index))
            cloudInliers->points.push_back(point);
        else
            NonGround->points.push_back(point);
    }
     
    KdTree* tree = new KdTree;
    std::vector<std::vector<float>> points;
    for(int i=0; i<(NonGround->points.size()); i++) 
    {
        tree->insert({NonGround->points[i].x, NonGround->points[i].y, NonGround->points[i].z}, i);
        points.push_back({NonGround->points[i].x, NonGround->points[i].y, NonGround->points[i].z});
    }
    std::vector<std::vector<int>> clusters_indices = euclideanCluster_pcd(points, tree, 0.4);
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clusters;  
    std::vector<Color> colors ={Color(1,0,0), Color(1,1,0),Color(0,0,1)};
    
    for(std::vector<int> cluster_indices : clusters_indices)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZI>());
        for(int indices: cluster_indices)
        {
            
                cluster->points.push_back(NonGround->points[indices]);
                cluster->width=cluster->points.size();
                cluster->height=1;
                cluster->is_dense=true;
        }
        if((cluster->points.size()> 30) && (cluster->points.size()<400))

        {clusters.push_back(cluster);}


    }
    int clusterid=0;

      for(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster_box : clusters)
        {
          //renderPointCloud(viewer, cluster_box,"obstacle"+std::to_string(clusterid),colors[clusterid%3] );  
          Box Bbox=processorI->BoundingBox(cluster_box); 
          renderBox(viewer, Bbox,clusterid,colors[clusterid%3],2);
          clusterid++;
        }
   
}


int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);
    //simpleHighway(viewer);

    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("../src/sensors/data/pcd/data_1");
    auto streamIterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;
    //cityBlock(viewer);

    
        while (!viewer->wasStopped ())
    {

    // Clear viewer
    viewer->removeAllPointClouds();
    viewer->removeAllShapes();

    // Load pcd and run obstacle detection process
    inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
    cityBlock(viewer, pointProcessorI, inputCloudI);

    streamIterator++;
    if(streamIterator == stream.end())
        streamIterator = stream.begin();

    viewer->spinOnce ();
    }
}