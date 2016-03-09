#include <pcl/io/pcd_io.h>
#include <pcl/keypoints/iss_3d.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/shot.h>
#include <ros/ros.h>
#include <ros/service.h> 
#include <iostream>
#include <pcl/correspondence.h>
#include <pcl/recognition/cg/geometric_consistency.h>
#include <pcl/filters/uniform_sampling.h>

 
// http://robotica.unileon.es/mediawiki/index.php/PCL/OpenNI_tutorial_5:_3D_object_recognition_%28pipeline%29

double
computeCloudResolution(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud)
{
	double resolution = 0.0;
	int numberOfPoints = 0;
	int nres;
	std::vector<int> indices(2);
	std::vector<float> squaredDistances(2);
	pcl::search::KdTree<pcl::PointXYZ> tree;
	tree.setInputCloud(cloud);
 
	for (size_t i = 0; i < cloud->size(); ++i)
	{
		if (! pcl_isfinite((*cloud)[i].x))
			continue;
 
		// Considering the second neighbor since the first is the point itself.
		nres = tree.nearestKSearch(i, 2, indices, squaredDistances);
		if (nres == 2)
		{
			resolution += sqrt(squaredDistances[1]);
			++numberOfPoints;
		}
	}
	if (numberOfPoints != 0)
		resolution /= numberOfPoints;
 
	return resolution;
}

void ISS_keypoints_detector ( pcl::PointCloud<pcl::PointXYZ>::Ptr cloud , pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints )
{
	// ISS keypoint detector object.
	pcl::ISSKeypoint3D<pcl::PointXYZ, pcl::PointXYZ> detector;
	detector.setInputCloud(cloud);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
	detector.setSearchMethod(kdtree);
	double resolution = computeCloudResolution(cloud);
	// Set the radius of the spherical neighborhood used to compute the scatter matrix.
	detector.setSalientRadius(6 * resolution);
	// Set the radius for the application of the non maxima supression algorithm.
	detector.setNonMaxRadius(4 * resolution);
	// Set the minimum number of neighbors that has to be found while applying the non maxima suppression algorithm.
	detector.setMinNeighbors(5);
	// Set the upper bound on the ratio between the second and the first eigenvalue.
	detector.setThreshold21(0.975);
	// Set the upper bound on the ratio between the third and the second eigenvalue.
	detector.setThreshold32(0.975);
	// Set the number of prpcessing threads to use. 0 sets it to automatic.
	detector.setNumberOfThreads(4);
 
	detector.compute(*keypoints);
}  


void downsample ( pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints, std::string ss)
{
  pcl::UniformSampling<pcl::PointXYZ> uniform_sampling;
  uniform_sampling.setInputCloud (cloud);
  if (ss == "model")
  uniform_sampling.setRadiusSearch (0.01f);
  else if (ss == "scene")
  uniform_sampling.setRadiusSearch (0.03f);
  
  uniform_sampling.filter (*keypoints);
  std::cout << "Model total points: " << cloud->size () << "; Selected Keypoints: " << keypoints->size () << std::endl;
}

void shot_descriptor (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normals, 
                      pcl::PointCloud<pcl::SHOT352>::Ptr descriptors)
{
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation;
	normalEstimation.setInputCloud(cloud);
	normalEstimation.setRadiusSearch(0.03);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
	normalEstimation.setSearchMethod(kdtree);
	normalEstimation.compute(*normals);
 
	// SHOT estimation object.
	pcl::SHOTEstimation<pcl::PointXYZ, pcl::Normal, pcl::SHOT352> shot;
	shot.setInputCloud(cloud);
	shot.setInputNormals(normals);
	// The radius that defines which of the keypoint's neighbors are described.
	// If too large, there may be clutter, and if too small, not enough points may be found.
	shot.setRadiusSearch(0.02);
 
	shot.compute(*descriptors);
}

void correspondence_matching ( pcl::PointCloud<pcl::SHOT352>::Ptr scene_descriptors, pcl::PointCloud<pcl::SHOT352>::Ptr model_descriptors,
                               pcl::CorrespondencesPtr correspondences)
{
  pcl::KdTreeFLANN<pcl::SHOT352> matching;
	matching.setInputCloud(model_descriptors);
	// A Correspondence object stores the indices of the query and the match,
	// and the distance/weight.
	
	// Check every descriptor computed for the scene.
	for (size_t i = 0; i < scene_descriptors->size(); ++i)
	{
		std::vector<int> neighbors(1);
		std::vector<float> squaredDistances(1);
		// Ignore NaNs.
		if (pcl_isfinite(scene_descriptors->at(i).descriptor[0]))
		{
			// Find the nearest neighbor (in descriptor space)...
			int neighborCount = matching.nearestKSearch(scene_descriptors->at(i), 1, neighbors, squaredDistances);
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

void geometric_consistency ( pcl::PointCloud<pcl::PointXYZ>::Ptr scene_keypoints, pcl::PointCloud<pcl::PointXYZ>::Ptr model_keypoints,
                             pcl::CorrespondencesPtr correspondences )
{
	
  std::cout<<"inside geometric clustering"<<std::endl;
  
  std::vector<pcl::Correspondences> clusteredCorrespondences;
	// Object for storing the transformations (rotation plus translation).
	std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > transformations;
  
  pcl::GeometricConsistencyGrouping<pcl::PointXYZ, pcl::PointXYZ> grouping;
	grouping.setSceneCloud(scene_keypoints);
	grouping.setInputCloud(model_keypoints);
	grouping.setModelSceneCorrespondences(correspondences);
	// Minimum cluster size. Default is 3 (as at least 3 correspondences
	// are needed to compute the 6 DoF pose).
	grouping.setGCThreshold(3);
	// Resolution of the consensus set used to cluster correspondences together,
	// in metric units. Default is 1.0.
	grouping.setGCSize(0.01);
 
	grouping.recognize(transformations, clusteredCorrespondences);
 
	std::cout << "Model instances found: " << transformations.size() << std::endl << std::endl;
	for (size_t i = 0; i < transformations.size(); i++)
	{
		std::cout << "Instance " << (i + 1) << ":" << std::endl;
		std::cout << "\tHas " << clusteredCorrespondences[i].size() << " correspondences." << std::endl << std::endl;
 
		Eigen::Matrix3f rotation = transformations[i].block<3, 3>(0, 0);
		Eigen::Vector3f translation = transformations[i].block<3, 1>(0, 3);
		printf("\t\t    | %6.3f %6.3f %6.3f | \n", rotation(0, 0), rotation(0, 1), rotation(0, 2));
		printf("\t\tR = | %6.3f %6.3f %6.3f | \n", rotation(1, 0), rotation(1, 1), rotation(1, 2));
		printf("\t\t    | %6.3f %6.3f %6.3f | \n", rotation(2, 0), rotation(2, 1), rotation(2, 2));
		std::cout << std::endl;
		printf("\t\tt = < %0.3f, %0.3f, %0.3f >\n", translation(0), translation(1), translation(2));
	}
  
  
}

int
main(int argc, char** argv)
{
  // Object for storing the point cloud.
  pcl::PointCloud<pcl::PointXYZ>::Ptr scene_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  // Object for storing the normals.
  pcl::PointCloud<pcl::Normal>::Ptr scene_normals(new pcl::PointCloud<pcl::Normal>);
  // Object for storing the SHOT descriptors for each point.
  pcl::PointCloud<pcl::SHOT352>::Ptr scene_descriptors(new pcl::PointCloud<pcl::SHOT352>());
  //
  pcl::PointCloud<pcl::PointXYZ>::Ptr scene_keypoints(new pcl::PointCloud<pcl::PointXYZ>);
  
  
      // Object for storing the point cloud.
      pcl::PointCloud<pcl::PointXYZ>::Ptr model_cloud(new pcl::PointCloud<pcl::PointXYZ>);
      // Object for storing the normals.
      pcl::PointCloud<pcl::Normal>::Ptr model_normals(new pcl::PointCloud<pcl::Normal>);
      // Object for storing the SHOT descriptors for each point.
      pcl::PointCloud<pcl::SHOT352>::Ptr model_descriptors(new pcl::PointCloud<pcl::SHOT352>());
      //
      pcl::PointCloud<pcl::PointXYZ>::Ptr model_keypoints(new pcl::PointCloud<pcl::PointXYZ>);
      
  pcl::CorrespondencesPtr correspondences(new pcl::Correspondences());
      
  // Read a PCD file from disk.
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1], *scene_cloud) != 0)
  {
    return -1;
  }
      // Read a PCD file from disk.
      if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[2], *model_cloud) != 0)
      {
        return -1;
      }
      
  //~ input cloud, output ISS keypoints
  ISS_keypoints_detector(scene_cloud, scene_keypoints);
  ISS_keypoints_detector(model_cloud, model_keypoints);
  
  downsample(scene_cloud, scene_keypoints,"scene");
  downsample(model_cloud, model_keypoints,"model");
  
  
  //~ input cloud, normal, output SHOT descriptors
  shot_descriptor(scene_cloud, scene_normals, scene_descriptors);
  shot_descriptor(model_cloud, model_normals, model_descriptors);                     
  // input 2xdescriptors, return correspondences
  correspondence_matching(scene_descriptors, model_descriptors, correspondences);    
  //~ input 2x keypoints, correspondences, std::cout R|T matrix
  geometric_consistency(scene_keypoints, model_keypoints, correspondences);
  
 std::cout<<"it is done"<<std::endl;
 
	
}
