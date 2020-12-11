cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

if(WIN32)
    # Note: PCLConfig.cmake fails to locate FindEigen.cmake
    # because it only has access tp FindEigen3.cmake
    # unless this path is added, with the necessary modules
    #list (APPEND CMAKE_MODULE_PATH C:/src/cmake_tools/modules)
    list (APPEND CMAKE_MODULE_PATH C:/src/PCL/pcl-1.11.1/pcl/cmake/Modules)
endif(WIN32)

project(pcd_surface)

if(WIN32)
    # WATCH OUT: Probably some of these need to be set again with ccmake / cmake-gui
    # To avoid it, put this before find_package(PCL REQUIRED)
    set(FLANN_INCLUDE_DIR C:/src/FLANN/flann_geomlib/src/cpp)
    set(FLANN_LIBRARY C:/src/FLANN/flann_geomlib/build/lib/Release/flann_cpp_s.lib)
    set(FLANN_LIBRARY_DEBUG C:/src/FLANN/flann_geomlib/build/lib/Debug/flann_cpp_s.lib)
    set(QHULL_LIBRARY C:/src/Qhull/qhull/bin/qhull_r.dll)
    set(QHULL_LIBRARY_DEBUG C:/src/Qhull/qhull/bin/qhull_rd.dll)
endif(WIN32)

find_package(PCL REQUIRED)

# Note: in modern CMake these are no longer necessary
# since they are called automatically
# depending on the libs linked with target_link_libraries()
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Executable 1: How to create a range image from a point cloud and a given sensor position.
# ./range_image_creation
add_executable (range_image_creation range_image_creation.cpp)
target_link_libraries (range_image_creation ${PCL_LIBRARIES})

# Executable 2: How to visualize a range image.
# ./range_image_visualization: pointcloud created and visualized in 2 windows: 3D pointcloud and 2D range image
# ./range_image_visualization -h: help
# ./range_image_visualization -l: interactive 2D range image generated from 3D pointcloud view modified with mouse
# ./range_image_visualization <my_pointcloud.pcd>: create range image of custom pointcloud; if pointcloud not passed, a synthetic unordered pointcloud is created
add_executable (range_image_visualization range_image_visualization.cpp)
target_link_libraries (range_image_visualization ${PCL_LIBRARIES})