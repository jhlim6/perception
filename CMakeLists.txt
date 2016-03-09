cmake_minimum_required(VERSION 2.8.3)
project(natural_features)

find_package(catkin REQUIRED COMPONENTS
  camera_info_manager
  cmake_modules
  cv_bridge
  eigen_conversions
  geometry_msgs
  message_generation
  pcl_conversions
  pcl_ros
  std_msgs
  tf_conversions
  image_transport
)

find_package(Boost REQUIRED COMPONENTS system filesystem thread)
find_package(Eigen REQUIRED)
find_package(PCL 1.8.0 REQUIRED)



catkin_package(
  CATKIN_DEPENDS 
    camera_info_manager
    cv_bridge
    eigen_conversions
    message_runtime
    geometry_msgs
    std_msgs
    pcl_conversions
    pcl_ros
    tf_conversions
)

# Order of the include dirs matters!
include_directories(include)
include_directories(${PCL_INCLUDE_DIRS}) 
include_directories(${catkin_INCLUDE_DIRS}) 
add_definitions(${PCL_DEFINITIONS})


# Libraries
add_executable(point_feature_histogram src/point_feature_histogram.cpp)
target_link_libraries(point_feature_histogram ${PCL_LIBRARIES} ${catkin_LIBRARIES})
#~ 
add_executable(fast_PFH src/fast_PFH.cpp)
target_link_libraries(fast_PFH ${PCL_LIBRARIES} ${catkin_LIBRARIES})

add_executable(descriptors_matching src/descriptors_matching.cpp)
target_link_libraries(descriptors_matching ${PCL_LIBRARIES} ${catkin_LIBRARIES})

add_executable(shot src/shot.cpp)
target_link_libraries(shot ${PCL_LIBRARIES} ${catkin_LIBRARIES})

add_executable(pipeline src/pipeline.cpp)
target_link_libraries(pipeline ${PCL_LIBRARIES} ${catkin_LIBRARIES})

add_executable(objrecon src/objrecon.cpp)
target_link_libraries(objrecon ${PCL_LIBRARIES} ${catkin_LIBRARIES})

add_executable(mesh2pcd src/mesh2pcd.cpp)
target_link_libraries(mesh2pcd ${PCL_LIBRARIES} ${catkin_LIBRARIES})

add_executable(mesh_sampling src/mesh_sampling.cpp)
target_link_libraries(mesh_sampling ${PCL_LIBRARIES} ${catkin_LIBRARIES})

add_executable(take_a_pcl_shot src/take_a_pcl_shot.cpp)
target_link_libraries(take_a_pcl_shot ${PCL_LIBRARIES} ${catkin_LIBRARIES})

