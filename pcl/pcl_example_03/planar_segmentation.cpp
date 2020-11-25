/* Simple program in which a point cloud is segmented in inliers that fit in a plane.
 * Segmentation is carried out using RANSAC.
 * Inlier indices and model parameters are extracted.
 * Look at:
 * https://pcl.readthedocs.io/projects/tutorials/en/latest/planar_segmentation.html#planar-segmentation
*/

#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

int main (int argc, char** argv) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

  // Fill in the cloud data
  cloud->width  = 15;
  cloud->height = 1;
  cloud->points.resize (cloud->width * cloud->height);

  // Generate the data: points on plane parallel to XY, Z = 1
  for (auto& point: *cloud)
  {
    point.x = 1024 * rand () / (RAND_MAX + 1.0f);
    point.y = 1024 * rand () / (RAND_MAX + 1.0f);
    point.z = 1.0;
  }

  // Set a few outliers
  (*cloud)[0].z = 2.0;
  (*cloud)[3].z = -2.0;
  (*cloud)[6].z = 4.0;

  std::cerr << "Point cloud data: " << cloud->size () << " points" << std::endl;
  for (const auto& point: *cloud)
    std::cerr << "    " << point.x << " "
                        << point.y << " "
                        << point.z << std::endl;

  // We define empty coefficients of a model and the inliers of the pointcloud that fit the model
  // The goal is to derive the parameters of the model and detect the inlier indices
  // The used model is a plane, but there are many more models defined in model_types.h
  // SACMODEL_PLANE, SACMODEL_LINE, SACMODEL_CIRCLE2D, SACMODEL_CIRCLE3D, SACMODEL_SPHERE, SACMODEL_CYLINDER, SACMODEL_CONE, ...
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  // The segmentation method is RANSAC; this is robust and simple (there are some other methods)
  seg.setMethodType (pcl::SAC_RANSAC);
  // Threshold: how close must inliers be?
  seg.setDistanceThreshold (0.01);

  seg.setInputCloud (cloud);
  // Segmentation of points according to the model occurs here
  seg.segment (*inliers, *coefficients);

  if (inliers->indices.size () == 0)
  {
    PCL_ERROR ("Could not estimate a planar model for the given dataset.");
    return (-1);
  }

  std::cerr << "Model coefficients: " << coefficients->values[0] << " " 
                                      << coefficients->values[1] << " "
                                      << coefficients->values[2] << " " 
                                      << coefficients->values[3] << std::endl;

  std::cerr << "Model inliers: " << inliers->indices.size () << std::endl;
  for (const auto& idx: inliers->indices)
    std::cerr << idx << "    " << cloud->points[idx].x << " "
                               << cloud->points[idx].y << " "
                               << cloud->points[idx].z << std::endl;

  return (0);
}