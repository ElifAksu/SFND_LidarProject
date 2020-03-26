/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting
#ifndef CLUSTER_PCD_H_
#define CLUSTER_PCD_H_

#include "render/render.h"
#include "render/box.h"


#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <chrono>
#include <string>
#include "kdtree.h"


// Arguments:
// window is the region to draw box around
// increase zoom to see more of the area

void Proximity2(std::vector<float> point,std::vector<int>& cluster,int point_id,std::vector<int> &status,KdTree* tree,float distanceTol);


std::vector<std::vector<int>> euclideanCluster_pcd(const std::vector<std::vector<float>>& points, KdTree* tree, float distanceTol);
 	

#endif
