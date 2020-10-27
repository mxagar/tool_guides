/* Simple program that computes the normals of an unordered pointcloud.
 * Look at:
 * https://pcl.readthedocs.io/projects/tutorials/en/latest/normal_estimation.html#normal-estimation
*/

#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
//#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <string> // string
#include <iostream> // cout
#include <math.h> // isnan

int main (int argc, char** argv) {

  // Watch out: Point cloud is declared as pointer
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

  // Read the pointcould
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

  // Create the normal estimation class, and pass the input dataset to it
  // Consider also the OpenMP parallelized version pcl::NormalEstimationOMP!
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  ne.setInputCloud (cloud);

  /*

  // Alternative 1: create a vector of indices and pass it to search only on them
  // Neighbors searched in whole cloud, normals computed in given indices
  std::vector<int> indices (std::floor (cloud->size () / 10));
  for (std::size_t i = 0; i < indices.size (); ++i) indices[i] = i;
  pcl::shared_ptr<std::vector<int> > indicesptr (new std::vector<int> (indices));
  ne.setInputCloud (cloud);
  ne.setIndices (indicesptr);

  // Alternative 2: pass the downsampled cloud as input
  // Neighbors searched in whole/surface cloud, computed input/downsampled cloud
  ne.setInputCloud (cloud_downsampled);
  ne.setSearchSurface (cloud);

  */

  // Create an empty kdtree representation, and pass it to the normal estimation object.
  // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
  ne.setSearchMethod (tree);

  // Output datasets
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

  // Use all neighbors in a sphere of radius 3cm
  // If this value is not correct, we can get NaNs, because there are no points in neighborhood...
  // If it is too big, more comutation time
  //ne.setRadiusSearch (0.03);
  
  // No matter how big the radius, 5 closest  points taken
  ne.setKSearch(5);
  
  // NOTE: if too much neighbor points are taken because of big radius or k, normals at edges ar enot sharp anymore!

  // Compute the features: in this case, the normals
  ne.compute (*cloud_normals);
  // Internal process steps: for each point
  // - Select neighbor points according to radius/k
  // - Compute covariance matrix and its eigen values & vectors
  // - Obtain normal and curvature from eigen data
  // - Flip/Select normal direction according to 2.5D image scanning viewpoint
  // IMPORTANT: the default viewpoint is (0,0,0), if that's not true, set anew: ne.setViewPoint(vx,vy,vy)

  // cloud_normals->size () should have the same size as the input cloud->size ()
std::cout << "Number of computed normals: " << cloud_normals->size() << std::endl;
double n_x = 0.0;
double n_y = 0.0;
double n_z = 0.0;
unsigned int count = 0;
for (auto& normal: *cloud_normals) {
  if (!isnan(normal.normal_x + normal.normal_y + normal.normal_z)) {
    n_x += double(normal.normal_x); n_y += double(normal.normal_y); n_z += double(normal.normal_z);
    count++;
  }
}
std::cout << "Number of CORRECT computed normals: " << count << std::endl;
std::cout << "Average normal direction: [ " << n_x/double(count) << ", "
                                          << n_y/double(count) << ", "
                                          << n_z/double(count) << " ]" << std::endl;

return 0;

}