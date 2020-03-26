#ifndef RANSAC_H_
#define RANSAC_H_


#include "render/render.h"
#include <unordered_set>

#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>

#include <iostream> 
#include <string>  
#include <vector>
#include <ctime>
#include <chrono>


std::unordered_set<int> Ransac3D_func(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, int maxIterations, float distanceTol);

#endif 