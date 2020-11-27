# PCL Tutorials

These projects explore PCL functionalities.

1. `pcl_example_00`: CMake compilation + basic point cloud is created and saved as *.pcd; the PCD format is also explained in the C++ file comments.

2. `pcl_example_01`: More advanced options for/with PCD pointclouds are explained

    - `new_point_type.cpp`: new point type is created and used to store a *.pcd.
    - `concatenate_clouds.cpp`: pointcouds are concatenated (point lists and fields).

3. `pcl_example_02`: Features are explored

    - PLY loaded: `normalEstimation.cpp`
    - **Visualization** of points and normals: `normalEstimationImages.cpp`, `pfhEstimation.cpp`
    - Normals computed in unordered pointcloud: `normalEstimation.cpp`
    - Normals computed in ordered pointcloud (XY relationship given): `normalEstimationImages.cpp`
    - PFHs computed in unordered pointcloud: `pfhEstimation.cpp`
    - Point pair normal fetures computed: `pfhEstimation.cpp`
    - Point neighborhood searched (kdtree): `pfhEstimation.cpp`
    - FPFHs computed in unordered pointcloud: `fpfhEstimation.cpp`

4. `pcl_example_03`: Filtering & Segmentation (= Selection)

    - Pointcloud segmentarion/filtering: points with Z in range are taken: `passthrough.cpp`
    - Pointcloud is embedded in voxel grid and for each voxel, the centroid of the points within it is delivered: `voxel_grid.cpp`
    - Statistical outlier removal targetting mainly noise in sparse regions; outliers in point Gaussian distribution are removed: `statistical_removal.cpp`
    - Points of a point cloud are projected on a primitive model (a plane): `project_inliers.cpp`
    - A pointcloud containing inliers & outliers of a model (plane or sphere) is segmented using RANSAC; basically, the inliers that fit the model (plane / sphere) are detected (and visualized): `random_sample_consensus.cpp`
    - A point cloud is segmented in inliers that fit in a plane. Segmentation is carried out using RANSAC. Inlier indices and model parameters are extracted: `planar_segmentation.cpp`
    - A cylindrical mug is segmented on a real scene with a table. It is an application that extends the example in `planar_segmentation.cpp` to cylinders in a real scenario: `cylinder_segmentation.cpp`
    - Several subsets of points which fit into different planes are extracted iteratively from a scene: `extract_segmented_indices.cpp`
    - Two types of point subsets are removed: (1) points that have less than n neighbor points in a radius R (argument -r), and (2) points that satisfy a set of simple conditions (argument -c): `remove_outliers.cpp`

    - Get the list of fields that a point has: `voxel_grid.cpp`
    - Print cloud object: `statistical_removal.cpp`
    - **Visualization** of points and coordinate ystem: `random_sample_consensus.cpp`
    - A `PointCloud2` data structure is created: `extract_segmented_indices.cpp`

5. `pcl_example_04`: Registration (= Matching) 

    - Iterative Closest Point (**ICP**) algorithm usage: `iterative_closest_point.cpp`
    - **Interactive ICP**: a point cloud is moved to a target position step by step using ICP and the process is **visualized**: `interactive_icp.cpp`
    - Two **large 3D scans** (360 rooms) are aligned using the Normal Distributions Transform (NDT) and **visualized**: `normal_distributions_transform.cpp`
    - The alignment pose of a rigid object in a **scene with clutter** and occlusions is found and **visualized**:: `alignment_prerejective.cpp`