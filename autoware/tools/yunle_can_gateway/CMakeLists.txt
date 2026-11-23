cmake_minimum_required(VERSION 3.16)
project(yunle_can_gateway LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(Boost REQUIRED COMPONENTS system)
find_package(Threads REQUIRED)

add_executable(yunle_can_gateway src/main.cpp)
target_compile_options(yunle_can_gateway PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(yunle_can_gateway PRIVATE Boost::system Threads::Threads)

include(GNUInstallDirs)
install(TARGETS yunle_can_gateway RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

