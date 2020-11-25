# PCL Tutorials

These projects explore PCL functionalities.

1. `pcl_example_00`: CMake compilation + basic point cloud is created and saved as *.pcd

2. `pcl_example_01`: new point type is created and used to store a *.pcd

3. `pcl_example_02`: Features are explored

    - PLY loaded: `normalEstimation.cpp`
    - Visualization of points and normals: `normalEstimationImages.cpp`, `pfhEstimation.cpp`
    - Normals computed in unordered pointcloud: `normalEstimation.cpp`
    - Normals computed in ordered pointcloud (XY relationship given): `normalEstimationImages.cpp`
    - PFHs computed in unordered pointcloud: `pfhEstimation.cpp`
    - Point pair normal fetures computed: `pfhEstimation.cpp`
    - Point neighborhood searched (kdtree): `pfhEstimation.cpp`
    - FPFHs computed in unordered pointcloud: `fpfhEstimation.cpp`

4. `pcl_example_03`: Filtering & Segmentation (= Selection)

    - Get the list of fields that a point has: `voxel_grid.cpp`
    - Print cloud object: `statistical_removal.cpp`
    - Visualization of points and coordinate ystem: `random_sample_consensus.cpp`
