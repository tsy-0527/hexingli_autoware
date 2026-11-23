cmake_minimum_required(VERSION 3.14)
project(autoware_lidar_camera_overlay)

find_package(ament_cmake REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)

add_executable(lidar_camera_overlay_node
  src/lidar_camera_overlay_node.cpp
)

target_include_directories(lidar_camera_overlay_node PRIVATE
  ${OpenCV_INCLUDE_DIRS}
)

ament_target_dependencies(lidar_camera_overlay_node
  cv_bridge
  message_filters
  rclcpp
  sensor_msgs
  tf2_ros
  tf2_sensor_msgs
)

target_link_libraries(lidar_camera_overlay_node
  ${OpenCV_LIBRARIES}
)

install(TARGETS lidar_camera_overlay_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
