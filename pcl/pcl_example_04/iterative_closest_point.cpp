/* Example in which a simple ICP is performed between artificial point clouds.
 * Look at:
 * https://pcl.readthedocs.io/projects/tutorials/en/latest/iterative_closest_point.html#iterative-closest-point
*/

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

int main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>(5,1));
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>);

  // Fill in the CloudIn data
  for (auto& point : *cloud_in)
  {
    point.x = 1024 * rand() / (RAND_MAX + 1.0f);
    point.y = 1024 * rand() / (RAND_MAX + 1.0f);
    point.z = 1024 * rand() / (RAND_MAX + 1.0f);
  }
  
  std::cout << "Saved " << cloud_in->size () << " data points to input:" << std::endl;
      
  for (auto& point : *cloud_in)
    std::cout << point << std::endl;

  // Cloud to align is initialized as target/model cloud 
  *cloud_out = *cloud_in;
  
  std::cout << "size:" << cloud_out->size() << std::endl;
  // We add 0.7 in X to cloud to align
  for (auto& point : *cloud_out)
    point.x += 0.7f;

  std::cout << "Transformed " << cloud_in->size () << " data points:" << std::endl;
      
  for (auto& point : *cloud_out)
    std::cout << point << std::endl;

  // Create ICP object and set source & target (model) clouds
  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  icp.setInputSource(cloud_in);
  icp.setInputTarget(cloud_out);
  
  // Perform the alignment; aligned cloud stored in Final
  pcl::PointCloud<pcl::PointXYZ> Final;
  icp.align(Final);

  // Extract results: converged, score, transformation
  std::cout << "ICP has converged: " << icp.hasConverged() << "\nFitness score: " <<  icp.getFitnessScore() << std::endl;
  // Transformation is identity + 0.7 in X
  std::cout << "Final transformation: \n" <<  icp.getFinalTransformation() << std::endl;

 return (0);
}