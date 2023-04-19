cmake_minimum_required(VERSION 3.0.2)
project(comm_test)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -Wall -g -std=c++20")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -std=c++20")

include(cmake/FetchDependencies.cmake)

catkin_package(CATKIN_DEPENDS
   roscpp
   std_msgs
   # cv_bridge
   # image_transport
   sensor_msgs
   LIBRARIES {PROJECT_NAME}
)

add_subdirectory(src)
