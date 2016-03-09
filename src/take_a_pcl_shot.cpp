// Standard headers
#include <string>
#include <fstream>
#include <ctime>
#include <iostream>
#include <sstream>
// ROS headers
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

// Image transport
#include <image_transport/image_transport.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// PCL headers
#include <pcl/common/colors.h>
#include <pcl/common/transforms.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <boost/foreach.hpp>
#include <pcl/PCLPointCloud2.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>

#include <pcl/range_image/range_image_planar.h>
#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/features/range_image_border_extractor.h>
#include <pcl/keypoints/narf_keypoint.h>
// PCL noise filtering
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>

//opencv
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

int i =0;
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
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*input,pcl_pc2);
    ROS_INFO("inside Callback");
    pcl::PCLPointCloud2::Ptr cloud_blob (new pcl::PCLPointCloud2 (pcl_pc2) ), cloud_filtered_blob (new pcl::PCLPointCloud2);
    *cloud_blob = pcl_pc2;
    pcl::PointCloud<pcl::PointXYZ> pcl_1;
    pcl::fromPCLPointCloud2(pcl_pc2, pcl_1);
    
  
  
  std::string ss = "ensenso_pcl_";
  
  std::stringstream ss1;
  ss1 << i++;
  ss += ss1.str();
  // ss.append( ss_temp);
  ss.append(".pcd");
  std::cout << ss << '\n';  
    
  pcl::io::savePCDFileASCII (ss, pcl_1);
    
  }

private:
  ros::NodeHandle nh; 
  ros::Publisher pub;
  ros::Subscriber sub;

};//End of class SubscribeAndPublish



void parseCommandLine (int argc, char *argv[])
{
  if (pcl::console::find_switch (argc, argv, "-v"))
  {
    
  }
  if (pcl::console::find_switch (argc, argv, "-s"))
  {
    
  }
  if (pcl::console::find_switch (argc, argv, "-r"))
  {
   
  }
}

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "cloud_sub");
  ROS_INFO("inside main");
  
  if (argc == 3 ){
    std::vector<int> filenames;
  filenames = pcl::console::parse_file_extension_argument (argc, argv, ".pcd");
  
  std::string model_filename_;
  std::string scene_filename_;
   pcl::PointCloud<pcl::PointXYZ>::Ptr model (new pcl::PointCloud<pcl::PointXYZ> ());
     pcl::PointCloud<pcl::PointXYZ>::Ptr scene (new pcl::PointCloud<pcl::PointXYZ> ());
  model_filename_ = argv[filenames[0]];
  scene_filename_ = argv[filenames[1]];

   pcl::io::loadPCDFile (model_filename_, *model);
  
    std::cout << "Model height and weight." << model->height <<" "<< model-> width << std::endl;
  
  pcl::io::loadPCDFile (scene_filename_, *scene);
    std::cout << "scene height and weight." << scene->height <<" "<< scene-> width << std::endl;
  
}
  
  parseCommandLine (argc, argv);
  SubscribeAndPublish SAPObject;

  ros::spin ();
}
