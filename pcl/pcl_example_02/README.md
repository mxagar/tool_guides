# Feature Computation in PCL

These examples are related to PCL feature computation.
For more info, see `pcl_guide.txt` or
[How PCL Features Work](https://pcl.readthedocs.io/projects/tutorials/en/latest/how_features_work.html#how-3d-features-work).

List of examples:

1. `normalEstimation.cpp`: Normals are estimated in an unordered pointcloud; see [Estimating Surface Normals in a PointCloud](https://pcl.readthedocs.io/projects/tutorials/en/latest/normal_estimation.html#normal-estimation).

2. `normalEstimationImages.cpp`: Normals are estimated in an ordered pointcloud created from depth images; [Normal Estimation Using Integral Images](https://pcl.readthedocs.io/projects/tutorials/en/latest/normal_estimation_using_integral_images.html#normal-estimation-using-integral-images).

3. `pfhEstimation.cpp`: Point Feature Histograms are generated; additionally, kdtrees are used to get the neighborhood indices of a given point. See [Point Feature Histograms (PFH) descriptors](https://pcl.readthedocs.io/en/latest/pfh_estimation.html).

4. `fpfhEstimation.cpp`: Point Feature Histograms are generated; see [Fast Point Feature Histograms (FPFH) descriptors](https://pcl.readthedocs.io/en/latest/fpfh_estimation.html).
