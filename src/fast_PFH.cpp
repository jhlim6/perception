#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh_omp.h>
#include <ros/ros.h>
#include <ros/service.h>
 

int
main(int argc, char** argv)
{
	// Object for storing the point cloud.
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	// Object for storing the normals.
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	// Object for storing the FPFH descriptors for each point.
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr descriptors(new pcl::PointCloud<pcl::FPFHSignature33>());
 
	// Read a PCD file from disk.
	if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1], *cloud) != 0)
	{
		return -1;
	}
 
	// Note: you would usually perform downsampling now. It has been omitted here
	// for simplicity, but be aware that computation can take a long time.
 
	// Estimate the normals.
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation;
	normalEstimation.setInputCloud(cloud);
	normalEstimation.setRadiusSearch(0.03);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
	normalEstimation.setSearchMethod(kdtree);
	normalEstimation.compute(*normals);
 
	// FPFH estimation object.
	pcl::FPFHEstimationOMP<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh;
	fpfh.setInputCloud(cloud);
	fpfh.setInputNormals(normals);
	fpfh.setSearchMethod(kdtree);
	// Search radius, to look for neighbors. Note: the value given here has to be
	// larger than the radius used to estimate the normals.
	fpfh.setRadiusSearch(0.05);
 
	fpfh.compute(*descriptors); 

  std::cout<<"FPFH descriptors done"<<std::endl;
  
  
  std::string ss_temp;
  ss_temp.append(argv[1]);
  std::size_t found = ss_temp.find_last_of("/\\");
  ss_temp = ss_temp.substr(found+1);
  std::string ss = "FPFH_";
  ss.append(ss_temp);
  std::cout << ss << '\n';  
 
  pcl::io::savePCDFileASCII (ss, *descriptors);
  
  
  
}
