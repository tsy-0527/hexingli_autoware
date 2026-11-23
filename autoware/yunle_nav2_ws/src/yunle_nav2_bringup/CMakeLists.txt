cmake_minimum_required(VERSION 3.8)
project(yunle_nav2_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  config
  launch
  maps
  rviz
  urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
