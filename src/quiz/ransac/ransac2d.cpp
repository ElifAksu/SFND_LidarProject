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
    return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
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
    float x1, x2, y1, y2;
    float A, B, C;
    // TODO: Fill in this function
    for(int i=0; i<maxIterations; i++){
    std::unordered_set<int> inliers;
    int indx1 = rand()%cloud->points.size();
    int indx2 = rand()%cloud->points.size();
    std::cout<<"index : "<< indx1 << " " <<indx2<<std::endl;
    x1= cloud->points[indx1].x; y1= cloud->points[indx1].y;
    x2= cloud->points[indx2].x; y2= cloud->points[indx2].y;
    std::cout<<"x values : "<< x1 << " " <<x2<<std::endl;
    A=(y1-y2); B=(x2-x1); C=(x1*y2-x2*y1);
        for(int index = 0; index < cloud->points.size(); index++)
        {
        pcl::PointXYZ point = cloud->points[index];
        float d = fabs(A*point.x +B*point.y+C)/sqrt(A*A+B*B);
            if(d<=distanceTol)
            {
                inliers.insert(index);
            }
        }
        if(inliers.size()>inliersResult.size())
        {
            inliersResult=inliers;
        }


    }
    // For max iterations 

    // Randomly sample subset and fit line

    // Measure distance between every point and fitted line
    // If distance is smaller than threshold count it as inlier

    // Return indicies of inliers from fitted line with most inliers
    
    return inliersResult;

}

std::unordered_set<int> Ransac3D(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
    std::unordered_set<int> inliersResult;
    
    srand(time(NULL));
    float x1, x2,x3, y1, y2,y3 ,z1, z2, z3;
    float A, B, C, D;
    
    // TODO: Fill in this function
    for(int i=0; i<maxIterations; i++){
    std::unordered_set<int> inliers;
    int indx1 = rand()%cloud->points.size();
    int indx2 = rand()%cloud->points.size();
    int indx3 = rand()%cloud->points.size();
    std::cout<<"index : "<< indx1 << " " <<indx2<<std::endl;
    x1= cloud->points[indx1].x; y1= cloud->points[indx1].y; z1=cloud->points[indx1].z;
    x2= cloud->points[indx2].x; y2= cloud->points[indx2].y; z2=cloud->points[indx2].z;
    x3= cloud->points[indx3].x; y3= cloud->points[indx3].y; z3=cloud->points[indx3].z;
    std::cout<<"x values : "<< x1 << " " <<x2<<" " <<x3<<std::endl;
    A=(y2-y1)*(z3-z1)-(z2-z1)*(y3-y1); B=(z2-z1)*(x3-x1)-(x2-x1)*(z3-z1); C=(x2-x1)*(y3-y1)-(y2-y1)*(x3-x1);
    D=-(A*x1+B*y1+C*z1);
        for(int index = 0; index < cloud->points.size(); index++)
        {
        pcl::PointXYZ point = cloud->points[index];
        float d = fabs(A*point.x +B*point.y+C*point.z +D)/sqrt(A*A+B*B+C*C);
            if(d<=distanceTol)
            {
                inliers.insert(index);
            }
        }
        if(inliers.size()>inliersResult.size())
        {
            inliersResult=inliers;
        }


    }
    // For max iterations 

    // Randomly sample subset and fit line

    // Measure distance between every point and fitted line
    // If distance is smaller than threshold count it as inlier

    // Return indicies of inliers from fitted line with most inliers
    
    return inliersResult;

}

int main ()
{

    // Create viewer
    //pcl::visualization::PCLVisualizer::Ptr viewer = initScene();
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    // Create data
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2D = CreateData();
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();
    

    // TODO: Change the max iteration and distance tolerance arguments for Ransac function
    //std::unordered_set<int> inliers = Ransac(cloud2D, 100,1);
    std::unordered_set<int> inliers = Ransac3D(cloud, 100,0.2);

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


    // Render 2D point cloud with inliers and outliers
    if(inliers.size())
    {
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