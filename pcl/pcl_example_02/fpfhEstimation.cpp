/* Simple program that computes the PFHs (Point Feature Histograms) for point clouds.
 * Look at:
 * https://pcl.readthedocs.io/en/latest/fpfh_estimation.html#fpfh-estimation
*/

// If PCL was compiled with the visualization module, uncommet this!
#ifndef VISUALIZE
//#define VISUALIZE
#endif

#ifdef VISUALIZE
#include <pcl/visualization/cloud_viewer.h>
#endif

#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/pfh.h>
#include <pcl/features/fpfh.h>
#include <pcl/kdtree/kdtree_flann.h>
//#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <string> // string
#include <iostream> // cout
#include <math.h> // isnan

int main (int argc, char** argv) {

  // -- 0. Load the pointcloud
  // Point cloud is declared as pointer
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  std::string filename;
  if (argc > 1) {
    filename = std::string(argv[1]);
  } else {
#ifdef _WIN32
    filename = "..\\..\\..\\data\\suzanne_blender_faces_points.ply";
#else
    filename = "../../data/suzanne_blender_faces_points.ply";
#endif
  }
  pcl::io::loadPLYFile(filename, *cloud);

  // -- 1. Normals need to be computed
  // See normalEstimation.cpp for insights
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  ne.setInputCloud (cloud);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
  ne.setSearchMethod (tree);
  pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
  ne.setKSearch(5);
  ne.compute (*normals);

  // -- 2. Compute PFHs
  // For a pointcloud of n points and k points per neighbor
  // creating PFH costs O(nk^2), which is not feasible in realtime
  // Fast PFHs are a simplification that can be computed in O(nk).
  // Main idea: instead of computing the histogram analyzing all pairs within a point neighborhood, compute in 2 steps
  // 1. first, for each query point Pq, compute [alpha, phi, theta, d] values with the points Pi inside its neighborhood N(Pq) and build histogram
  //   that is the Simplified PFH: SPFH(Pq)
  //   in other words, Pq-Pi pairs are processed not all Pi-Pi pairs
  // 2. after all points SPFH(Pq) have been computed, sum the weighted histograms of neighborhood points
  //   FPFH(Pq) = SPFH(Pq) + (1/k)*sum_k((1/wi)*SPFH(Pi))
  //   wi = dist(Pq,Pi)
  // While for PFH we had pcl::PFHSignature125
  // now, we have pcl::FPFHSignature33
  //   11 bins for each of the 3 angle values
  //   it is not b^f, as with PFH, because now each feature is decoupled, so: b*f

  // Create the FPFH estimation class, and pass the input dataset+normals to it
  pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh;
  fpfh.setInputCloud (cloud);
  fpfh.setInputNormals (normals);
  // alternatively, if cloud is of type PointNormal, do fpfh.setInputNormals (cloud);

  // Create an empty kdtree representation, and pass it to the FPFH estimation object.
  // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
  //pcl::search::KdTree<PointXYZ>::Ptr tree (new pcl::search::KdTree<PointXYZ>);
  fpfh.setSearchMethod (tree);

  // Output datasets
  pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs (new pcl::PointCloud<pcl::FPFHSignature33> ());

  // If we are unsure of the radius search sizes, we can compute the AABB and the diagonal
  // Bounding box computation
  float x_min = 0.0, x_max = 0.0, y_min = 0.0, y_max = 0.0, z_min = 0.0, z_max = 0.0;
  for (auto& point: *cloud) {
    x_min = point.x < x_min ? point.x : x_min;
    x_max = point.x > x_max ? point.x : x_max;
    y_min = point.y < y_min ? point.y : y_min;
    y_max = point.y > y_max ? point.y : y_max;
    z_min = point.z < z_min ? point.z : z_min;
    z_max = point.z > z_max ? point.z : z_max;
  }
  float diagonal = (x_max-x_min)*(x_max-x_min) + (y_max-y_min)*(y_max-y_min) + (z_max-z_min)*(z_max-z_min);
  diagonal = sqrt(diagonal);
  std::cout << "Cloud diagonal: " << diagonal
                                    << "; dx,dy,dz: "
                                    << (x_max-x_min) << ", "
                                    << (y_max-y_min) << ", "
                                    << (z_max-z_min) << std::endl;

  // Use all neighbors in a sphere of radius 5cm/0.05 units
  // IMPORTANT: the radius used here has to be larger than the radius used to estimate the surface normals!!!
  //fpfh.setRadiusSearch (0.02*diagonal);
  fpfh.setRadiusSearch (0.05);

  // Due to efficiency, PFHEstimation doesn't check normals
  // If we know there are NaN values, we need to fix them somehow before calling to compute()
  float v = 1.0 / sqrt(3.0);
  for (auto& normal: *normals) {
    if (isnan(normal.normal_x + normal.normal_y + normal.normal_z)) {
        normal.normal_x = v; normal.normal_y = v; normal.normal_z = v;
    }
  }

  // Compute the features
  fpfh.compute (*fpfhs);

  // pfhs->size() should be = cloud->size()
  std::cout << "Size of FPFH: " << fpfhs->size()
            << ", Size of cloud: " << cloud->size()
            << ", Size of normals: " << normals->size() << std::endl;

  // Visualize points
#ifdef VISUALIZE
  pcl::visualization::PCLVisualizer viewer("PCL Viewer");
  viewer.setBackgroundColor (0.0, 0.0, 0.5);
  viewer.addPointCloudNormals<pcl::PointXYZ,pcl::Normal>(cloud, normals);
  
  while (!viewer.wasStopped ())
  {
    viewer.spinOnce ();
  }
#endif

return 0;

}