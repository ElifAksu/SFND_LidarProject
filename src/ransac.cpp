#include "ransac.h"



std::unordered_set<int> Ransac3D_func(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, int maxIterations, float distanceTol)
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
    //std::cout<<"index : "<< indx1 << " " <<indx2<<std::endl;
    x1= cloud->points[indx1].x; y1= cloud->points[indx1].y; z1=cloud->points[indx1].z;
    x2= cloud->points[indx2].x; y2= cloud->points[indx2].y; z2=cloud->points[indx2].z;
    x3= cloud->points[indx3].x; y3= cloud->points[indx3].y; z3=cloud->points[indx3].z;
    //std::cout<<"x values : "<< x1 << " " <<x2<<" " <<x3<<std::endl;
    A=(y2-y1)*(z3-z1)-(z2-z1)*(y3-y1); B=(z2-z1)*(x3-x1)-(x2-x1)*(z3-z1); C=(x2-x1)*(y3-y1)-(y2-y1)*(x3-x1);
    D=-(A*x1+B*y1+C*z1);
        for(int index = 0; index < cloud->points.size(); index++)
        {
        pcl::PointXYZI point = cloud->points[index];
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
   
    return inliersResult;

}

