# Registration (Matching) in PCL

These examples are related to registration in PCL.
Filtering can be also understood as **matching**.
For more info, see `pcl_guide.txt` or
[The PCL Registration API](https://pcl.readthedocs.io/projects/tutorials/en/latest/registration_api.html#registration-api).

List of examples:

1. `iterative_closest_point.cpp`: A simple ICP is performed between artificial point clouds; see [How to use iterative closest point](https://pcl.readthedocs.io/projects/tutorials/en/latest/iterative_closest_point.html#iterative-closest-point).

2. `interactive_icp.cpp`: A point cloud is moved to a target position step by step using ICP and the process is **visualized**; see [Interactive Iterative Closest Point](https://pcl.readthedocs.io/projects/tutorials/en/latest/interactive_icp.html#interactive-icp).

3. `normal_distributions_transform.cpp`: Example in which 2 large 3D scans (360 rooms) are aligned using the Normal Distributions Transform (NDT); results are **visualized**; see [How to use Normal Distributions Transform](https://pcl.readthedocs.io/projects/tutorials/en/latest/normal_distributions_transform.html#normal-distributions-transform)

4. `alignment_prerejective.cpp`: Example in which the alignment pose of a rigid object in a scene with clutter and occlusions is found; results are **visualized**; see [Robust pose estimation of rigid objects](https://pcl.readthedocs.io/projects/tutorials/en/latest/alignment_prerejective.html#alignment-prerejective)
