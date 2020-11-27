# Surface processing (e.g., Triangulation) in PCL

These examples are related to surface processing (e.g., triangulation) in PCL.
For more info, see `pcl_guide.txt`.

List of examples:

1. `resampling.cpp`: Moving Least Squares (MLS) surface reconstruction method is used to smooth and resample noisy data with errors in normals due to alignment, among others; see [Smoothing and normal estimation based on polynomial reconstruction](https://pcl.readthedocs.io/projects/tutorials/en/latest/resampling.html#moving-least-squares).

2. `concave_convex_hull_2d.cpp`: A simple 2D hull polygon (concave or convex) for a set of points supported by a plane; see [Construct a concave or convex hull polygon for a plane model](https://pcl.readthedocs.io/projects/tutorials/en/latest/hull_2d.html#hull-2d)

3. `greedy_projection.cpp`: Triangulation is performed locally, by projecting the local neighborhood of a point along the point’s normal, and connecting unconnected points; see [Fast triangulation of unordered point clouds](https://pcl.readthedocs.io/projects/tutorials/en/latest/greedy_projection.html#greedy-triangulation)
