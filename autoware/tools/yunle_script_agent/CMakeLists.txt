cmake_minimum_required(VERSION 3.16)
project(yunle_script_agent LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(Boost REQUIRED COMPONENTS system)
find_package(Threads REQUIRED)

add_executable(yunle_script_agent src/main.cpp)
target_compile_options(yunle_script_agent PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(yunle_script_agent PRIVATE Boost::system Threads::Threads)

