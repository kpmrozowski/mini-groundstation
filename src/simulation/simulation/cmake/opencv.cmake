include(${CMAKE_DIR}/fetchcontent_declare.cmake)

function(add_opencv_modules MODULES)
    foreach(the_module ${MODULES})
        if(TARGET ${the_module})
            set_target_properties(
                ${the_module}
                PROPERTIES
                    INTERFACE_INCLUDE_DIRECTORIES
                        $<TARGET_PROPERTY:${the_module},INCLUDE_DIRECTORIES>
            )
            string(REPLACE "opencv_" "" module_name ${the_module})
            add_library(OpenCV::${module_name} ALIAS ${the_module})
        endif()
    endforeach()
endfunction(add_opencv_modules)

function(print_recognized_opencv_modules MODULES)
    foreach(the_module ${MODULES})
        Message(${the_module})      
    endforeach()
endfunction(print_recognized_opencv_modules)

set(VERSION 4.5.3)

if(USE_SYSTEM_LIBS)
    find_package(OpenCV ${VERSION})

    if(OpenCV_FOUND)
        add_opencv_modules("${OpenCV_LIB_COMPONENTS}")
        include_directories(${OpenCV_INCLUDE_DIRS})
        return()
    endif()
endif()

FetchContent_Declare(
    opencv
    GIT_REPOSITORY https://github.com/opencv/opencv.git
    GIT_TAG ${VERSION}
)
FetchContent_Declare_GH(opencv_contrib opencv/opencv_contrib 4.5.3)

FetchContent_GetProperties(opencv)
if(NOT opencv_POPULATED)
    FetchContent_Populate(opencv)
    FetchContent_Populate(opencv_contrib)

    set(WITH_EIGEN ON)
    set(WITH_OPENEXR OFF)
    set(WITH_ITT OFF)
    set(BUILD_TESTS OFF)
    set(BUILD_PERF_TESTS OFF)
    set(BUILD_TIFF ON)
    set(BUILD_WITH_STATIC_CRT ${STATIC_CRT})
    set(BUILD_SHARED_LIBS OFF)
    set(WITH_FFMPEG OFF)
    set(WITH_GSTREAMER OFF)
    set(WITH_GTK OFF)
    set(WITH_QUIRC OFF)
    set(OPENCV_EXTRA_MODULES_PATH ${opencv_contrib_SOURCE_DIR}/modules CACHE PATH "Where to look for additional OpenCV modules (can be ;-separated list of paths)" FORCE)

    # we whitelist only modules that we use, to check list of all modules that are present call print_recognized_opencv_modules 
    # print_recognized_opencv_modules("${OPENCV_MODULES_BUILD}")
    set(BUILD_LIST core,imgcodecs,imgproc,calib3d,videoio,highgui,stereo,rgbd,shape,surface_matching,viz,ximgproc,cudevcudaimgproc,,cudaarithm,cudawarping,cudabgsegm,cudacodec,cudastereo)

    add_subdirectory(${opencv_SOURCE_DIR} ${opencv_BINARY_DIR} EXCLUDE_FROM_ALL)

    add_opencv_modules("${OPENCV_MODULES_BUILD}")

    get_directory_property(JPEG_LIBRARY DIRECTORY ${opencv_SOURCE_DIR} DEFINITION JPEG_LIBRARY)
    if(TARGET ${JPEG_LIBRARY})
        set_target_properties(${JPEG_LIBRARY}
            PROPERTIES
                INTERFACE_INCLUDE_DIRECTORIES
                    $<TARGET_PROPERTY:${JPEG_LIBRARY},INCLUDE_DIRECTORIES>
        )
        set(CMAKE_DISABLE_FIND_PACKAGE_JPEG TRUE)
        set(JPEG_FOUND TRUE)
        add_library(JPEG::JPEG ALIAS ${JPEG_LIBRARY})
    endif()

    get_directory_property(PNG_LIBRARY DIRECTORY ${opencv_SOURCE_DIR} DEFINITION PNG_LIBRARY)
    if(TARGET ${PNG_LIBRARY})
        set_target_properties(${PNG_LIBRARY}
            PROPERTIES
                INTERFACE_INCLUDE_DIRECTORIES
                    $<TARGET_PROPERTY:${PNG_LIBRARY},INCLUDE_DIRECTORIES>
        )
        set(CMAKE_DISABLE_FIND_PACKAGE_PNG TRUE)
        set(PNG_FOUND TRUE)
        add_library(PNG::PNG ALIAS ${PNG_LIBRARY})
    endif()
endif()
