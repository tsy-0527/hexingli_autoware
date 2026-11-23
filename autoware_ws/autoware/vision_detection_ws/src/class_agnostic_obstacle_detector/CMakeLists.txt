cmake_minimum_required(VERSION 3.14)
project(class_agnostic_obstacle_detector)

find_package(ament_cmake REQUIRED)
find_package(autoware_perception_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tier4_perception_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(class_agnostic_obstacle_detector_node
  src/class_agnostic_obstacle_detector_node.cpp
)
target_compile_features(class_agnostic_obstacle_detector_node PUBLIC cxx_std_17)
ament_target_dependencies(class_agnostic_obstacle_detector_node
  autoware_perception_msgs
  cv_bridge
  geometry_msgs
  rclcpp
  sensor_msgs
  std_msgs
  tier4_perception_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)
target_link_libraries(class_agnostic_obstacle_detector_node ${OpenCV_LIBRARIES})

install(TARGETS class_agnostic_obstacle_detector_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
