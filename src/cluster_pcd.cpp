/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "render/render.h"
#include "render/box.h"


#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <chrono>
#include <string>
#include "cluster_pcd.h"

// Arguments:
// window is the region to draw box around
// increase zoom to see more of the area

void Proximity2(int indice, const std::vector<std::vector<float>>& points,std::vector<int>& cluster,std::vector<bool>& status,KdTree* tree,float distanceTol)
{
	status[indice]=true;
	cluster.push_back(indice);
	std::vector<int> near_point = tree->search(points[indice], distanceTol);
	for(int index : near_point)
	{
		if(!status[index])
		{
			status[index] =1;
	  		Proximity2(index,points,cluster, status,tree,distanceTol);

		}
	}


}

std::vector<std::vector<int>> euclideanCluster_pcd(const std::vector<std::vector<float>>& points, KdTree* tree, float distanceTol)
{

	// TODO: Fill out this function to return list of indices for each cluster

	std::vector<std::vector<int>> clusters;
	std::vector<bool> status(points.size(),false);
	int i=0;
	while(i<points.size())
	{
		std::vector<float> point=points[i];
		if(status[i])
		{
			i++;
			continue;
		}
		
		
			std::vector<int> cluster;
			Proximity2(i,points,cluster,status,tree,distanceTol);
			clusters.push_back(cluster);
			i++;


		
		

	}

 
	return clusters;

}

