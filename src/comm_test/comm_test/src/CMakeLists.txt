# OpenCV
# find_package(OpenCV 4.4)
# if(NOT OpenCV_FOUND)
#    message(FATAL_ERROR "OpenCV > 4.4 not found.")
# endif()
# MESSAGE("OPENCV VERSION in play_volleyball: ${OpenCV_VERSION}")

# play_volleyball node
add_executable(play_volleyball
   volleyball_node.cpp
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

target_include_directories(play_volleyball PRIVATE
    ${catkin_INCLUDE_DIRS}
    ${fmt_SOURCE_DIR}/include
    ${spdlog_INCLUDE_DIR}
)

# Build ROS nodes
# list(FILTER catkin_LIBRARIES EXCLUDE REGEX ".*libopencv.*\.so\.4.2.0")

set(play_volleyball_LIBS
   ${catkin_LIBRARIES}
   # ${OpenCV_LIBS}
   # /usr/local/lib/libopencv_core.so.3.4.5
   # /usr/local/lib/libopencv_imgproc.so.3.4.5
   # /usr/local/lib/libopencv_video.so.3.4.5
   # /usr/local/lib/libopencv_datasets.so.3.4.5
   fmt::fmt-header-only
   spdlog::spdlog_header_only
)

target_link_libraries(play_volleyball PUBLIC
   ${play_volleyball_LIBS}
)

if (OpenCV_FOUND)
   add_definitions(-DOPEN_CV_INSTALLED)
endif()
