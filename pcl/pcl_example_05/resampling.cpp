/* Moving Least Squares (MLS) surface reconstruction method is used to smooth and resample noisy data with errors in normals due to alignment, among others.
 * Look at:
 * https://pcl.readthedocs.io/projects/tutorials/en/latest/resampling.html#moving-least-squares
*/

#include <pcl/point_types.h>
// Although not in the tutorial web, I need to add these 2 headers to compile...
#include <pcl/features/normal_3d.h>
#include <pcl/features/pfh.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>

int main(int argc, char **argv)
{

  // Moving Least Squares (MLS) surface reconstruction method is used to smooth and resample noisy data with errors in normals due to alignment, among others.
  // By performing resampling, small errors in normal direction(eg, due to alignment of several views) can be corrected and the “double walls” artifacts resulted from registering multiple scans together can be smoothed.
  // Resampling recreates the missing parts of the surface by higher order polynomial interpolations between the surrounding data points.

  // Load input file into a PointCloud<T> with an appropriate type
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());

  // Load bun0.pcd
  // Download from:
  // https://raw.githubusercontent.com/PointCloudLibrary/pcl/master/test/bun0.pcd
  pcl::io::loadPCDFile("../../data/bun0.pcd", *cloud);

  // Create a KD-Tree for searching neighbor points
  // I need to change this line to compile...
  //pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());

  // Output has the PointNormal type in order to store the normals calculated by MLS
  pcl::PointCloud<pcl::PointNormal> mls_points;

  // Init MLS object (second point type is for the normals, even if unused)
  pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;

  mls.setComputeNormals(true);

  // Set parameters
  mls.setInputCloud(cloud);
  mls.setPolynomialOrder(2); // this could be skipped
  mls.setSearchMethod(tree);
  mls.setSearchRadius(0.03);

  // Reconstruct
  mls.process(mls_points);

  // Save output
  pcl::io::savePCDFile("bun0-mls.pcd", mls_points);
  // To visualize:
  // pcl_viewer bun0-mls.pcd
}