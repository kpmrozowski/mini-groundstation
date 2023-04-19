project(offline_genertor)

set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_C_FLAGS_DEBUG "-O0 -Wall")
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -Wall -std=c++17")
set(CMAKE_C_FLAGS_RELEASE "-O0 -Wall")
set(CMAKE_CXX_FLAGS_RELEASE "-O0 -Wall -g -std=c++17")

# YAML
find_package(yaml-cpp REQUIRED)

# OpenCV
find_package(OpenCV 4.2.0 REQUIRED
   NO_MODULE #Should be optional, tells CMake to use config mode
   PATHS /usr/share # Tells CMake to look here
   NO_DEFAULT_PATH #and don't look anywhere else
)
if(NOT OpenCV_FOUND)
   message(FATAL_ERROR "OpenCV > 4.2.0 not found.")
endif()
MESSAGE("OPENCV VERSION in offline_genertor: ${OpenCV_VERSION}")

# CUDA
set(CUDA_TOOLKIT_ROOT_DIR /usr/local/cuda-11.6)
find_package(CUDA REQUIRED)
if(NOT CUDA_FOUND)
   message(FATAL_ERROR "CUDA 11.6 not found.")
else()
   message("km97 CUDA Found")
endif()

# Eigen3
find_package(Eigen3 3.1.0 REQUIRED)
MESSAGE("Eigen3 VERSION: ${Eigen3_VERSION}")

# add LibSGM
include(../../cmake/FindLibSGM.cmake)

# add nlohmann_json
include(../../cmake/fetchcontent_declare.cmake)
include(../../cmake/json.cmake)

# ============ generate_depth executable ============
add_executable(generate_depth
    ../core/SharedData.cpp
    generate_depth.cpp
    CsvReader.cpp
    DataAggregator.cpp
    utils.cpp
)

target_include_directories(generate_depth PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../common
    ${CMAKE_CURRENT_SOURCE_DIR}/../core
    ${catkin_INCLUDE_DIRS}
    ${LIBSGM_INCLUDE_DIRS}
    ${fmt_SOURCE_DIR}/include
    ${spdlog_INCLUDE_DIR}
)

target_link_libraries(generate_depth PUBLIC
    ${OpenCV_LIBS}
    stdc++fs
    yaml-cpp
    sgm
    ${CUDA_LIBRARIES}
    orb_slam_wrapper
    fmt::fmt-header-only
    spdlog::spdlog_header_only
    nlohmann_json::nlohmann_json
)

target_link_directories(generate_depth PUBLIC ${PCL_LIBRARY_DIRS})
target_compile_definitions(generate_depth PUBLIC ${PCL_DEFINITIONS})
