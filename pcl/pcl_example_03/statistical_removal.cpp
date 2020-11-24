/* Simple program which performs statistical outlier removal targetting mainly noise in sparse regions.
 * The distance distribution (assumed Gaussian) of a point wrt its neighbor points is computed and points outside of it are removed.
 * Look at:
 * https://pcl.readthedocs.io/projects/tutorials/en/latest/statistical_outlier.html#statistical-outlier-removal
*/

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>

int main (int argc, char** argv) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

  // Fill in the cloud data
  pcl::PCDReader reader;

  // Download file:
  // https://raw.github.com/PointCloudLibrary/data/master/tutorials/table_scene_lms400.pcd
  reader.read<pcl::PointXYZ> ("../../../data/table_scene_lms400.pcd", *cloud);

  // Note: point cloud has << overloaded operator for output!
  std::cerr << "Cloud before filtering: " << std::endl;
  std::cerr << *cloud << std::endl;

  // The SOR class creates the distance distribution of a point wrt its neighbors (k nearest neighbors must be found)
  // The distribution is assumed Gaussian
  // All points with a distance further away than the number of std deviations specified are removed
  // Effect: noise is removed, mainly sparse fat tails
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor; // Stands for Statistical Outlier Removal
  sor.setInputCloud (cloud);
  sor.setMeanK (50); // neighborhoods of 50 nearest points computed
  sor.setStddevMulThresh (1.0); // points witha distance higher/smalled than mean +- 1.0*stddev removed
  sor.filter (*cloud_filtered);

  std::cerr << "Cloud after filtering: " << std::endl;
  std::cerr << *cloud_filtered << std::endl;

  pcl::PCDWriter writer;
  writer.write<pcl::PointXYZ> ("table_scene_lms400_inliers.pcd", *cloud_filtered, false);

  sor.setNegative (true);
  sor.filter (*cloud_filtered);
  writer.write<pcl::PointXYZ> ("table_scene_lms400_outliers.pcd", *cloud_filtered, false);

  return (0);
}
