cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

if(WIN32)
    # Note: PCLConfig.cmake fails to locate FindEigen.cmake
    # because it only has access tp FindEigen3.cmake
    # unless this path is added, with the necessary modules
    #list (APPEND CMAKE_MODULE_PATH C:/src/cmake_tools/modules)
    list (APPEND CMAKE_MODULE_PATH C:/src/PCL/pcl-1.11.1/pcl/cmake/Modules)
    #list (APPEND CMAKE_MODULE_PATH C:/src/PCL/pcl-1.10.0/cmake/Modules)
endif(WIN32)

project(pcd_registration)

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

# Executable 1: A simple ICP is performed between artificial point clouds.
# ./iterative_closest_point
add_executable (iterative_closest_point iterative_closest_point.cpp)
target_link_libraries (iterative_closest_point ${PCL_LIBRARIES})

# Executable 2: A point cloud is moved to a target position step by step using ICP and the process is visualized
# WARNING: This example works only with the visualization module!
# ./interactive_icp ../../data/suzanne_blender_faces_points.ply
add_executable (interactive_icp interactive_icp.cpp)
target_link_libraries (interactive_icp ${PCL_LIBRARIES})

# Executable 3: Two large 3D scans (360 rooms) are aligned using the Normal Distributions Transform (NDT).
# WARNING: This example works only with the visualization module!
# ./normal_distributions_transform
add_executable (normal_distributions_transform normal_distributions_transform.cpp)
target_link_libraries (normal_distributions_transform ${PCL_LIBRARIES})

# Executable 4: Example in which the alignment pose of a rigid object in a scene with clutter and occlusions is found.
# WARNING: This example works only with the visualization module!
# ./alignment_prerejective ../../data/chef.pcd ../../data/rs1.pcd
add_executable (alignment_prerejective alignment_prerejective.cpp)
target_link_libraries (alignment_prerejective ${PCL_LIBRARIES})