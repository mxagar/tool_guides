cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

if(WIN32)
    # Note: PCLConfig.cmake fails to locate FindEigen.cmake
    # because it only has access tp FindEigen3.cmake
    # unless this path is added, with the necessary modules
    #list (APPEND CMAKE_MODULE_PATH C:/src/cmake_tools/modules)
    list (APPEND CMAKE_MODULE_PATH C:/src/PCL/pcl-1.11.1/pcl/cmake/Modules)
endif(WIN32)

project(pcd_write)

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

# Executable 1: Pointcloud segmentarion/filtering: points with Z in range are taken.
# ./passthrough
add_executable (passthrough passthrough.cpp)
target_link_libraries (passthrough ${PCL_LIBRARIES})

# Executable 2: Pointcloud is embedded in voxel grid and for each voxel, the centroid of the points within it is delivered.
# ./voxel_grid
add_executable (voxel_grid voxel_grid.cpp)
target_link_libraries (voxel_grid ${PCL_LIBRARIES})

# Executable 3: Statistical outlier removal targetting mainly noise in sparse regions; outliers in point Gaussian distribution are removed.
# ./statistical_removal
add_executable (statistical_removal statistical_removal.cpp)
target_link_libraries (statistical_removal ${PCL_LIBRARIES})

# Executable 4: Points of a point cloud are projected on a primitive model (a plane).
# ./project_inliers
add_executable (project_inliers project_inliers.cpp)
target_link_libraries (project_inliers ${PCL_LIBRARIES})

# Executable 5: A pointcloud containing inliers & outliers of a model (plane or sphere) is segmented using RANSAC; basically, the inliers that fit the model (plane / sphere) are detected (and visualized).
# ./random_sample_consensus <|-s|-f|-sf>
add_executable (random_sample_consensus random_sample_consensus.cpp)
target_link_libraries (random_sample_consensus ${PCL_LIBRARIES})

# Executable 6: A point cloud is segmented in inliers that fit in a plane. Segmentation is carried out using RANSAC. Inlier indices and model parameters are extracted.
# ./planar_segmentation
add_executable (planar_segmentation planar_segmentation.cpp)
target_link_libraries (planar_segmentation ${PCL_LIBRARIES})

# Executable 7: A cylindrical mug is segmented on a real scene with a table. It is an application that extends the example in `planar_segmentation.cpp` to cylinders in a real scenario.
# ./cylinder_segmentation 
add_executable (cylinder_segmentation cylinder_segmentation.cpp)
target_link_libraries (cylinder_segmentation ${PCL_LIBRARIES})

# Executable 8: Several subsets of points which fit into different planes are extracted iteratively from a scene.
# ./extract_segmented_indices
add_executable (extract_segmented_indices extract_segmented_indices.cpp)
target_link_libraries (extract_segmented_indices ${PCL_LIBRARIES})

# Executable 9: Two types of point subsets are removed: (1) points that have less than n neighbor points in a radius R (argument -r), and (2) points that satisfy a set of simple conditions (argument -c).
# ./remove_outliers <-c|-r>
add_executable (remove_outliers remove_outliers.cpp)
target_link_libraries (remove_outliers ${PCL_LIBRARIES})