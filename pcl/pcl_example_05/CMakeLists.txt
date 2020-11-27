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

# Executable 1: Moving Least Squares (MLS) surface reconstruction method is used to smooth and resample noisy data with errors in normals due to alignment, among others.
# ./resampling
# pcl_viewer bun0-mls.pcd 
add_executable (resampling resampling.cpp)
target_link_libraries (resampling ${PCL_LIBRARIES})

# Executable 2: A simple 2D hull polygon (concave or convex) for a set of points supported by a plane
# ./concave_convex_hull_2d [-concave]
# pcl_viewer table_scene_mug_stereo_textured_hull.pcd
add_executable (concave_convex_hull_2d concave_convex_hull_2d.cpp)
target_link_libraries (concave_convex_hull_2d ${PCL_LIBRARIES})

# Executable 3: 
# ./greedy_projection
# pcl_vtk2pcd mesh.vtk mesh.pcd
# pcl_viewer mesh.pcd
add_executable (greedy_projection greedy_projection.cpp)
target_link_libraries (greedy_projection ${PCL_LIBRARIES})