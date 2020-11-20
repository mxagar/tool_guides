/* Simple program that computes the PFHs (Point Feature Histograms) for point clouds.
 * Look at:
 * https://pcl.readthedocs.io/en/latest/pfh_estimation.html
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
  // Each point ends up having a PFH descriptor which is the distribution of how the normals change in its neighborhood
  // How PFH descriptors are computed:
  // - Given a point, its neighborhood is computed
  // - For each point pair in the neighborhood (quadratic), features [alpha, phi, theta, d] are computed
  //    . alpha, phi, theta = angle differences between normals
	//	  . d = distance between points
  //    . in order to compute the normal angle differences, a coordinate system is defined for each pair using the normals and their distance vector
  //    . NOTE: usually the feature d is not used (because efficiency and less benefit)
	// - a histogram is built for each point + neighborhood with b^f bins
  //    . b: number of ranges, usually 5
  //    . f: number of feautes: 3 if [alpha, beta, gama], 4 if [alpha, beta, gama]
  //    . therefore, usually we have 5^3 = 125 bins
  //    . in each bin, we count how many occurrences appear for the point neighborhood
  //    . IMPORTANT: we have like a sphere devided in 125 sectors and count how many occurrences happen in each sector

  // Create the PFH estimation class, and pass the input dataset + normals to it
  pcl::PFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::PFHSignature125> pfh;
  pfh.setInputCloud (cloud);
  pfh.setInputNormals (normals);
  // alternatively, if cloud is of type PointNormal, do pfh.setInputNormals (cloud);

  // If not done yet, create an empty kdtree representation, and pass it to the PFH estimation object.
  // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
  //pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
  pfh.setSearchMethod (tree);

  // Output datasets
  pcl::PointCloud<pcl::PFHSignature125>::Ptr pfhs (new pcl::PointCloud<pcl::PFHSignature125> ());

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

  // Use all neighbors in a sphere of radius 5cm / 0.05 units
  // IMPORTANT: the radius used here has to be larger than the radius used to estimate the surface normals!!!
  pfh.setRadiusSearch (0.05);

  // Due to efficiency, PFHEstimation doesn't check normals
  // If we know there are NaN values, we need to fix them somehow before calling to compute()
  float v = 1.0 / sqrt(3.0);
  for (auto& normal: *normals) {
    if (isnan(normal.normal_x + normal.normal_y + normal.normal_z)) {
        normal.normal_x = v; normal.normal_y = v; normal.normal_z = v;
    }
  }

  // Compute the features
  pfh.compute (*pfhs);

  // pfhs->size() should be = cloud->size()
  std::cout << "Size of PFH: " << pfhs->size()
            << ", Size of cloud: " << cloud->size()
            << ", Size of normals: " << normals->size() << std::endl;

  // -- 3. Further functions related to PFHs
  
  // If we want to compute the [alpha, phi, theta, d] features of a point pair:
  // pcl::computePairFeatures() or pfh.computePairFeatures()
  int index_1 = 0;
  int index_2 = 1;
  // Is there a faster way to convert PCL<->Eigen structures??
  Eigen::Vector4f p1, p2, n1, n2;
  p1[0] = ((*cloud)[index_1]).x;
  p1[1] = ((*cloud)[index_1]).y;
  p1[2] = ((*cloud)[index_1]).z;
  p1[3] = 0.0;
  p2[0] = ((*cloud)[index_2]).x;
  p2[1] = ((*cloud)[index_2]).y;
  p2[2] = ((*cloud)[index_2]).z;
  p2[3] = 0.0;
  n1[0] = ((*normals)[index_1]).normal_x;
  n1[1] = ((*normals)[index_1]).normal_y;
  n1[2] = ((*normals)[index_1]).normal_z;
  n1[3] = 0.0;
  n2[0] = ((*normals)[index_2]).normal_x;
  n2[1] = ((*normals)[index_2]).normal_y;
  n2[2] = ((*normals)[index_2]).normal_z;
  n2[3] = 0.0;
  float f1, f2, f3, f4;

  pcl::computePairFeatures(p1, n1, p2, n2, f1, f2, f3, f4);
  std::cout << "Point pair features (pcl): " << f1 << ", " << f2 << ", " << f3 << ", " << f4 << std::endl;

  pfh.computePairFeatures(*cloud, *normals, index_1, index_2, f1, f2, f3, f4);
  std::cout << "Point pair features (pfh): " << f1 << ", " << f2 << ", " << f3 << ", " << f4 << std::endl;

  // If we want to compute the PFH descriptor/signature of a neighborhood
  // pfh.computePointPFHSignature()
  // BUT: It is commented out, because I havent managed to make it work...
  int nr_split = 5;
  Eigen::VectorXf pfh_histogram;

  // First, we need to create a vector of nieghborhood indices
  // We could just take some indices, or we can do it properly by using a KDTree
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  kdtree.setInputCloud(cloud);
  pcl::PointXYZ searchPoint;
  int searchIndex = 0;
  searchPoint.x = (*cloud)[searchIndex].x;
  searchPoint.y = (*cloud)[searchIndex].y;
  searchPoint.z = (*cloud)[searchIndex].z;

  // K nearest neighbor search
  int K = 10;
  std::vector<int> pointIdxNKNSearch(K);
  std::vector<float> pointNKNSquaredDistance(K);
  if (kdtree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
  {
    // std::cout << "Nearest k indices: " << pointIdxNKNSearch[0] << std::endl;
    // pfh.computePointPFHSignature (*cloud, *normals, pointIdxNKNSearch, nr_split, pfh_histogram);
    // std::cout << "Histogram size: " << pfh_histogram.size() << std::endl;
    // std::cout << "Histogram sum " <<  pfh_histogram.sum() << std::endl;
    // std::cout << "Histogram 0, 1, 2: " <<  pfh_histogram[0] << ", " << pfh_histogram[1] << ", " << pfh_histogram[2] << std::endl;
  }

  // Neighbors within radius search
  std::vector<int> pointIdxRadiusSearch;
  std::vector<float> pointRadiusSquaredDistance;
  float radius = 0.1*diagonal;
  if (kdtree.radiusSearch (searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
  {
    //std::cout << "Nearest indices within r: " << pointIdxRadiusSearch[0] << std::endl;
    // pfh.computePointPFHSignature (*cloud, *normals, pointIdxRadiusSearch, nr_split, pfh_histogram);
    // std::cout << "Histogram size: " << pfh_histogram.size() << std::endl;
    // std::cout << "Histogram sum " <<  pfh_histogram.sum() << std::endl;
    // std::cout << "Histogram 0, 1, 2: " <<  pfh_histogram[0] << ", " << pfh_histogram[1] << ", " << pfh_histogram[2] << std::endl;
  }

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