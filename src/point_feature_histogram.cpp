#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/pfh.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <ros/ros.h>
#include <ros/service.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/console/parse.h>
 #include <pcl/visualization/cloud_viewer.h>

bool readfile = false;
bool use_ensenso = false;
bool save_file = false;
int PFH ( pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

class SubscribeAndPublish
{
public:
  SubscribeAndPublish()
  {
    
    pub = nh.advertise<std_msgs::Float64MultiArray>("/interest_point", 1);

    //Topic you want to subscribe
    sub = nh.subscribe("/ensenso/depth/points", 2, &SubscribeAndPublish::callback, this);
  }

  void callback(const boost::shared_ptr<const sensor_msgs::PointCloud2>& input)
  {
    pcl::PCLPointCloud2 cloud_2;
    pcl_conversions::toPCL(*input,cloud_2);
    
    ROS_INFO("inside Callback");
    pcl::PCLPointCloud2::Ptr cloud_blob (new pcl::PCLPointCloud2 (cloud_2) );
    *cloud_blob = cloud_2;
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_1_ptr (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ> cloud_1;
    pcl::fromPCLPointCloud2(cloud_2, cloud_1);
    
    *cloud_1_ptr = cloud_1;
    PFH(cloud_1_ptr);
  }

private:
  ros::NodeHandle nh; 
  ros::Publisher pub;
  ros::Subscriber sub;

};//End of class SubscribeAndPublish



int parseCommandLine (int argc, char *argv[])
{
  if (pcl::console::find_switch (argc, argv, "-readfile"))
  {
    readfile = true;
    // Read a PCD file from disk.
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    
   if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1], *cloud) != 0)
    {
      return -1;
    }
    ROS_INFO("inside readfile, going to prcoess PFH for input cloud");
    PFH(cloud);

  }
  if (pcl::console::find_switch (argc, argv, "-use_ensenso"))
  {
    use_ensenso = true;
    SubscribeAndPublish SAPObject;
  }
  if (pcl::console::find_switch (argc, argv, "-save_file"))
  {
    save_file = true;
  }
}
 
int PFH ( pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	// Object for storing the point cloud.
	// pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	// Object for storing the normals.
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	// Object for storing the PFH descriptors for each point.
	pcl::PointCloud<pcl::PFHSignature125>::Ptr descriptors(new pcl::PointCloud<pcl::PFHSignature125>());
  
 /*   pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
    viewer.showCloud (cloud);
    pcl::io::savePCDFileASCII ("test_pcd.pcd", *cloud); */



  
	// Note: you would usually perform downsampling now. It has been omitted here
	// for simplicity, but be aware that computation can take a long time.
 
	// Estimate the normals.
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation;
	normalEstimation.setInputCloud(cloud);
	normalEstimation.setRadiusSearch(0.03);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
	normalEstimation.setSearchMethod(kdtree);
	normalEstimation.compute(*normals);
 
	// PFH estimation object.
	pcl::PFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::PFHSignature125> pfh;
	pfh.setInputCloud(cloud);
	pfh.setInputNormals(normals);
	pfh.setSearchMethod(kdtree);
	// Search radius, to look for neighbors. Note: the value given here has to be
	// larger than the radius used to estimate the normals.
	pfh.setRadiusSearch(0.05);
 
	pfh.compute(*descriptors);
  ROS_INFO("Done");
}

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "cloud_sub");
  ROS_INFO("inside main");
  parseCommandLine (argc, argv);
  //SubscribeAndPublish SAPObject;

  ros::spin ();
}
