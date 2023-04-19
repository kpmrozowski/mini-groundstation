# YAML
find_package(yaml-cpp REQUIRED)

# OpenCV
find_package(OpenCV 4.2.0)
if(NOT OpenCV_FOUND)
   message(FATAL_ERROR "OpenCV > 4.2.0 not found.")
endif()
MESSAGE("OPENCV VERSION in collect_data: ${OpenCV_VERSION}")

# CUDA
set(CUDA_TOOLKIT_ROOT_DIR /usr/local/cuda-11.6)
find_package(CUDA REQUIRED)
if(NOT CUDA_FOUND)
   message(FATAL_ERROR "CUDA 11.6 not found.")
else()
   message("km97 CUDA Found")
endif()

set(SOURCE_FILES "")
FILE(GLOB SOURCE_FILES *.hpp *.cpp
   ${SAMPLES_LINUX_DIR}/common/dji_linux_environment.cpp
   ${SAMPLES_LINUX_DIR}/common/dji_linux_helpers.cpp
   ${SAMPLES_LINUX_DIR}/hal/*.c
   ${SAMPLES_LINUX_DIR}/osal/*.c
)

# collect_data node
add_executable(collect_data
   ${SOURCE_FILES}
)

find_package(catkin REQUIRED COMPONENTS
   roscpp
   std_msgs
   # cv_bridge
   # image_transport
   tf
   sensor_msgs
   message_generation
)

target_include_directories(collect_data PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../orb_slam_wrapper
    ${CMAKE_CURRENT_SOURCE_DIR}/../common
    ${catkin_INCLUDE_DIRS}
    ${OSDK_INCLUDE_DIRS}
    ${LIBSGM_INCLUDE_DIRS}
    ${LIBUSB_1_INCLUDE_DIRS}
    ${FFMPEG_INCLUDE_DIR}
    ${fmt_SOURCE_DIR}/include
    ${spdlog_INCLUDE_DIR}
)

# Build ROS nodes

target_link_libraries(collect_data PUBLIC
   ${catkin_LIBRARIES}
   ${OpenCV_LIBS}
   djiosdk-core
   advanced-sensing
   stdc++fs
   yaml-cpp
   sgm
   ${CUDA_LIBRARIES}
   orb_slam_wrapper
   fmt::fmt-header-only
   spdlog::spdlog_header_only
)
if (OpenCV_FOUND)
   add_definitions(-DOPEN_CV_INSTALLED)
endif()
