#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/shot.h>
#include <ros/ros.h>
#include <ros/service.h> 
#include <iostream>
 
int
main(int argc, char** argv)
{
	// Object for storing the SHOT descriptors for the scene.
	pcl::PointCloud<pcl::SHOT352>::Ptr sceneDescriptors(new pcl::PointCloud<pcl::SHOT352>());
	// Object for storing the SHOT descriptors for the model.
	pcl::PointCloud<pcl::SHOT352>::Ptr modelDescriptors(new pcl::PointCloud<pcl::SHOT352>());
 
	// Read the already computed descriptors from disk.
	if (pcl::io::loadPCDFile<pcl::SHOT352>(argv[1], *sceneDescriptors) != 0)
	{
		ROS_INFO("scene Descriptor failed");
    return -1;
	}
	if (pcl::io::loadPCDFile<pcl::SHOT352>(argv[2], *modelDescriptors) != 0)
	{
    ROS_INFO("model Descriptor failed");
		return -1;
	}
 
	// A kd-tree object that uses the FLANN library for fast search of nearest neighbors.
	pcl::KdTreeFLANN<pcl::SHOT352> matching;
	matching.setInputCloud(modelDescriptors);
	// A Correspondence object stores the indices of the query and the match,
	// and the distance/weight.
	pcl::CorrespondencesPtr correspondences(new pcl::Correspondences());
 
	// Check every descriptor computed for the scene.
	for (size_t i = 0; i < sceneDescriptors->size(); ++i)
	{
		std::vector<int> neighbors(1);
		std::vector<float> squaredDistances(1);
		// Ignore NaNs.
		if (pcl_isfinite(sceneDescriptors->at(i).descriptor[0]))
		{
			// Find the nearest neighbor (in descriptor space)...
			int neighborCount = matching.nearestKSearch(sceneDescriptors->at(i), 1, neighbors, squaredDistances);
			// ...and add a new correspondence if the distance is less than a threshold
			// (SHOT distances are between 0 and 1, other descriptors use different metrics).
			if (neighborCount == 1 && squaredDistances[0] < 0.25f)
			{
				pcl::Correspondence correspondence(neighbors[0], static_cast<int>(i), squaredDistances[0]);
				correspondences->push_back(correspondence);
			}
		}
	}
	std::cout << "Found " << correspondences->size() << " correspondences." << std::endl;
}
