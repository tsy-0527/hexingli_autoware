cmake_minimum_required(VERSION 3.8)
project(gins2000)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Gins2000.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

find_package(serial REQUIRED)
include_directories(include ${serial_INCLUDE_DIRS})

# The package name "serial" also exists as an unrelated system library.  Resolve
# the ROS workspace library from the prefix containing serialConfig.cmake so the
# linker cannot accidentally select /usr/lib/.../libserial.so.
get_filename_component(serial_prefix "${serial_DIR}/../../.." ABSOLUTE)
find_library(gins2000_serial_library
  NAMES serial
  PATHS "${serial_prefix}/lib"
  NO_DEFAULT_PATH
  REQUIRED
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)

add_executable(gins2000_node src/gins2000_node.cpp)
target_link_libraries(gins2000_node ${gins2000_serial_library} ${cpp_typesupport_target})
ament_target_dependencies(gins2000_node rclcpp geometry_msgs)

add_executable(gins2000_mock_publisher src/gins2000_mock_publisher.cpp)
target_link_libraries(gins2000_mock_publisher ${cpp_typesupport_target})
ament_target_dependencies(gins2000_mock_publisher rclcpp geometry_msgs)

install(TARGETS
  gins2000_node
  gins2000_mock_publisher
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch rviz
  DESTINATION share/${PROJECT_NAME}
)

# ── 安装 Python 脚本 ──────────────────────────────────────
install(PROGRAMS
  scripts/converter_node.py
  scripts/gnss_ins_converter.py
  DESTINATION lib/${PROJECT_NAME}
)

# ── 安装 Python 脚本 ──────────────────────────────────────
install(PROGRAMS
  scripts/odom_to_twist.py
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()
