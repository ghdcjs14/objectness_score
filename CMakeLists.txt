cmake_minimum_required(VERSION 2.8.3)
project(objectness_score)

#SET (CMAKE_BUILD_TYPE Release)
#SET(CMAKE_CXX_FLAGS -O3)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  nav_msgs
  sensor_msgs
  geometry_msgs
  message_generation
  gazebo_ros
  roscpp
  rospy
  tf2
  tf2_ros
  tf2_msgs
  roslib
  actionlib
  move_base
  cv_bridge
  image_transport
  pcl_ros
  pcl_conversions
)

find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)
find_package(PCL 1.9 REQUIRED COMPONENTS common io)

# Declare ROS messages and services
add_message_files(
  FILES
  Result.msg
  Matrix.msg
)

generate_messages(DEPENDENCIES std_msgs sensor_msgs)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES objectness_score
  CATKIN_DEPENDS gazebo_ros roscpp rospy std_msgs nav_msgs sensor_msgs tf2 tf2_ros tf2_msgs cv_bridge opencv2 pcl_ros pcl_conversions # dependency of catkin packages
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS} 
  ${PCL_INCLUDE_DIRS}
)

add_executable(objectness_score src/objectness_score.cpp src/Collector.cpp include/objectness_score/Collector.h src/Projector.cpp include/objectness_score/Projector.h)
add_dependencies(objectness_score ${${PROJECT_NAME}_EXPORTED_TARGETS}${catkin_EXPORTED_TARGETS})
target_link_libraries(
  objectness_score
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES} 
  ${PCL_LIBRARIES}
)
