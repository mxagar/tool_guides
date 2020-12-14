# PCL Tutorials

These projects explore PCL functionalities.
Important: use tools like `pcl_viewer` along with the examples, if possible.

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
    - Point neighborhood searched (kd-tree): `pfhEstimation.cpp`
    - FPFHs computed in unordered pointcloud: `fpfhEstimation.cpp`

4. `pcl_example_03`: Filtering & Segmentation (= Selection)

    - Pointcloud segmentarion/filtering: points with Z in range are taken: `passthrough.cpp`
    - Pointcloud is embedded in voxel grid and for each voxel, the centroid of the points within it is delivered: `voxel_grid.cpp`
    - Statistical outlier removal targetting mainly noise in sparse regions; outliers in point Gaussian distribution are removed: `statistical_removal.cpp`
    - Points of a point cloud are projected on a primitive model (a plane): `project_inliers.cpp`
    - A pointcloud containing inliers & outliers of a model (plane or sphere) is segmented using RANSAC; basically, the inliers that fit the model (plane / sphere) are detected (and **visualized**): `random_sample_consensus.cpp`
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
    - **Rigid 3D Transformations** with `pcl::transformPointCloud(*cloud_in, *cloud_out, trafo)`: `interactive_icp.cpp`
    - Two **large 3D scans** (360 rooms) are aligned using the Normal Distributions Transform (NDT) and **visualized**: `normal_distributions_transform.cpp`
    - The alignment pose of a rigid object in a **scene with clutter** and occlusions is found and **visualized**:: `alignment_prerejective.cpp`

6. `pcl_example_05`: Surface processing (e.g., Triangulation)

    - Moving Least Squares (MLS) surface reconstruction method smooth and resample noisy data with errors in normals due to alignment, among others: `resampling.cpp`
    - Create a 2D convex / concave hull (3D points that enclose the hull) of a planar point cloud; a simple 2D hull polygon (concave or convex) for a set of points supported by a plane: `concave_convex_hull_2d.cpp`
    - Greedy surface triangulation algorithm on a PointCloud with normals, to obtain a triangle mesh based on projections of the local neighborhoods. Triangulation is performed locally, by projecting the local neighborhood of a point along the point’s normal, and connecting unconnected points: `greedy_projection.cpp`

7. `pcl_example_06`: Visualization

    - Simple visualization blueprint with the `CloudViewer` class - but it doesn't work on my macloudViewer: `cloud_viewer.cpp`
    - Simple visualization utils compiled by me, eg., pointcloud with and without normals, point size, background color, coordinate system, plane, cone, sphere: `simple_viewer.cpp`
    - Several examples with the `pcl::visualization::PCLVisualizer` class, the main visualization class in PCL -- many elements are visualized: `pcl_visualizer_demo.cpp`

    - See also the PCL tool `pcl_viewer`

    - Additional examples in other folders that visualize elements:
        - normals are visualized: `pcl_example_02/fpfhEstimation.cpp`, `pcl_example_02/normalEstimationImages.cpp`, `pcl_example_02/pfhEstimation.cpp`
        - pointcloud and coordinate system are visualized: `pcl_example_03/random_sample_consensus.cpp`
        - pointclouds with different single colors are shown after alignment: `pcl_example_04/alignment_prerejective.cpp`, `pcl_example_04/normal_distributions_transform.cpp`
        - **very intersting example**: pointclouds with different single colors are shown in 2 viewports as they are interactively aligned: `pcl_example_04/interactive_icp.cpp`
        - range image visualization: `pcl_example_07/range_image_visualization.cpp`, `pcl_example_07/range_image_border_extraction.cpp`, `pcl_example_07/narf_keypoint_extraction.cpp`

8. `pcl_example_07`: Other useful data structures: Range Images and Search Structures (kd-trees & octrees)

    - Range images: how to create them from point clouds (a synthetic **unordered** is used), given a sensor position: `range_image_creation.cpp`
    - How to **visualize** range images: unordered pointcloud can be loaded (or created, if not specified one), its range image created and visualized; 2 windows are opened: (1) the 3D pointcloud and the (2) 2D range image, which can be interactively updated with the movement of the 3D pointcloud: `range_image_visualization.cpp`
    - Range image border extraction: given a pointcloud (if not given, a synthetic one is created), three boder points are detected and **visualized**: **border**, **shadow**, **veil**;`range_image_border_extraction.cpp`
    - NARF keypoints on range images: `narf_keypoint_extraction.cpp`
    - NARF features on range images: `narf_feature_extraction.cpp`
    - kd-tree
    - octree

9. `pcl_example_08`: Recognition

    - 3D Object Recognition based on Correspondence Grouping
    - Implicit Shape Model
    - Tutorial: Hypothesis Verification for 3D Object Recognition
