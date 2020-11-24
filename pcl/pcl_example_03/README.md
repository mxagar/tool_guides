# Filtering and Segmentation in PCL

These examples are related to filtering in PCL.
Filtering can be also understood as **segmentation** or **selection**.
For more info, see `pcl_guide.txt` or
[Filtering in PCL](https://pcl.readthedocs.io/projects/tutorials/en/latest/index.html#filtering).

List of examples:

1. `passthrough.cpp`: cut off or select/segment values that are either inside or outside a given user range; see [Passthrough Filtering in a PointCloud](https://pcl.readthedocs.io/projects/tutorials/en/latest/passthrough.html#passthrough).

2. `voxel_grid.cpp`: pointcloud is embedded in voxel grid and for each voxel, the centroid of the points within it is delivered; see [Downsampling a PointCloud using a VoxelGrid filter](https://pcl.readthedocs.io/projects/tutorials/en/latest/voxel_grid.html#voxelgrid).

3. `statistical_removal.cpp`: statistical outlier removal targetting mainly noise in sparse regions; the distance distribution (assumed Gaussian) of a point wrt its neighbor points is computed and points outside of it are removed; see [Removing outliers using a StatisticalOutlierRemoval filter](https://pcl.readthedocs.io/projects/tutorials/en/latest/statistical_outlier.html#statistical-outlier-removal)

4. `project_inliers.cpp`: points of a point cloud are projected on a primitive model (a plane); see [Projecting points using a parametric model](https://pcl.readthedocs.io/projects/tutorials/en/latest/project_inliers.html#project-inliers).

5. `planar_segmentation.cpp`

6. `cylinder_segmentation.cpp`

7. `extract_segmented_indices.cpp`

8. `remove_outliers.cpp`