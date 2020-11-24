/* Simple program that cuts off values that are either inside or outside a given user range.
 * Filtering consists here in segmenting points within a range in a dimension/field, eg. X, Y, Z.
 * Look at:
 * https://pcl.readthedocs.io/projects/tutorials/en/latest/passthrough.html#passthrough
*/

#include <iostream>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>

int main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

  // Fill in the cloud data
  cloud->width  = 10;
  cloud->height = 1; // unordered pointcloud
  cloud->points.resize (cloud->width * cloud->height);

  for (auto& point: *cloud)
  {
    point.x = 1024 * rand () / (RAND_MAX + 1.0f);
    point.y = 1024 * rand () / (RAND_MAX + 1.0f);
    point.z = 1024 * rand () / (RAND_MAX + 1.0f);
  }

  std::cerr << "Cloud before filtering: " << std::endl;
  for (const auto& point: *cloud)
    std::cerr << "    " << point.x << " "
                        << point.y << " "
                        << point.z << std::endl;

  // Create the filtering object and select the dimension/field in which we'd like to filter/segment
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("z"); //x, y, z
  pass.setFilterLimits (0.0, 5.0);
  //pass.setFilterLimitsNegative (true); // deprecated, use following method
  pass.setNegative(true); // if active, the points that lie in the complementary range (limits) are selected
  pass.filter (*cloud_filtered);

  std::cerr << "Cloud after filtering: " << std::endl;
  for (const auto& point: *cloud_filtered)
    std::cerr << "    " << point.x << " "
                        << point.y << " "
                        << point.z << std::endl;

  return (0);
}