cmake_minimum_required(VERSION 3.14)
project(gnss_ins_localizer)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake_auto REQUIRED)
find_package(yaml-cpp REQUIRED)

ament_auto_find_build_dependencies()

find_library(GeographicLib_LIBRARIES NAMES Geographic)
if(NOT GeographicLib_LIBRARIES)
  message(FATAL_ERROR "GeographicLib not found. sudo apt install libgeographic-dev")
endif()

ament_auto_add_library(${PROJECT_NAME}_component SHARED
  src/gnss_ins_localizer.cpp
)
target_link_libraries(${PROJECT_NAME}_component
  ${GeographicLib_LIBRARIES}
  yaml-cpp
)

rclcpp_components_register_node(${PROJECT_NAME}_component
  PLUGIN "gnss_ins_localizer::GnssInsLocalizer"
  EXECUTABLE ${PROJECT_NAME}_node
)

install(PROGRAMS
  scripts/fake_gins_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

ament_auto_package(
  USE_SCOPED_HEADER_INSTALL_DIR
  INSTALL_TO_SHARE
    config
    launch
)
