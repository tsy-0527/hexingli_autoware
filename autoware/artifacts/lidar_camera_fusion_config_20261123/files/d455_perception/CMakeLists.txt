cmake_minimum_required(VERSION 3.8)
project(d455_perception)
find_package(ament_cmake REQUIRED)
set(DETECTOR_PYTHON "/usr/bin/python3" CACHE FILEPATH "Python interpreter with torch and ROS 2 dependencies")
configure_file(scripts/detect.py.in detect @ONLY)
configure_file(scripts/custom_roi_detector.py.in custom_roi_detector @ONLY)
configure_file(scripts/roi_merger.py.in roi_merger @ONLY)
file(CHMOD "${CMAKE_CURRENT_BINARY_DIR}/custom_roi_detector"
  PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE
              GROUP_READ GROUP_EXECUTE
              WORLD_READ WORLD_EXECUTE)
file(CHMOD "${CMAKE_CURRENT_BINARY_DIR}/roi_merger"
  PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE
              GROUP_READ GROUP_EXECUTE
              WORLD_READ WORLD_EXECUTE)
install(PROGRAMS "${CMAKE_CURRENT_BINARY_DIR}/detect" DESTINATION lib/${PROJECT_NAME})
install(
  PROGRAMS
    "${CMAKE_CURRENT_BINARY_DIR}/custom_roi_detector"
    "${CMAKE_CURRENT_BINARY_DIR}/roi_merger"
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY runtime DESTINATION share/${PROJECT_NAME} PATTERN "__pycache__" EXCLUDE PATTERN "*.pyc" EXCLUDE)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
ament_package()
