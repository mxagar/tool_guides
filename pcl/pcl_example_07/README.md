# Additional Data Structures in PCL: Range Images and Their Keypoints, KdTrees, Octrees

These examples are related to additional and useful data structuctures in PCL. Additional to the ones in the example folders `00` - `06`.
Roughly, following data structures are covered:

- Range images: how to create and visualize them, and how to extract keypoints in them.
- Search structures: kd-trees and octrees.

For more info, see `pcl_guide.txt`.

List of examples:

1. `range_image_creation.cpp`: how to create a range image from a point cloud (a synthetic **unordered** is used) and a given sensor position; see [How to create a range image from a point cloud](https://pcl.readthedocs.io/projects/tutorials/en/latest/range_image_creation.html#range-image-creation).

2. `range_image_visualization.cpp`: how to visualize range images: unordered pointcloud can be loaded (or created, if not specified one), its range image created and visualized; 2 windows are opened: (1) the 3D pointcloud and the (2) 2D range image, which can be interactively updated with the movement of the 3D pointcloud; see [How to visualize a range image](https://pcl.readthedocs.io/projects/tutorials/en/latest/range_image_visualization.html#range-image-visualization).
