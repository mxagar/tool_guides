/* Simple program in which the points of a point cloud are projected on a primitive model (a plane).
 * So, the inliers are really the projected points.
 * Look at:
 * https://pcl.readthedocs.io/projects/tutorials/en/latest/project_inliers.html#project-inliers
*/

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>

int
 main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected (new pcl::PointCloud<pcl::PointXYZ>);

  // Fill in the cloud data
  cloud->width  = 5;
  cloud->height = 1;
  cloud->points.resize (cloud->width * cloud->height);

  for (auto& point: *cloud)
  {
    point.x = 1024 * rand () / (RAND_MAX + 1.0f);
    point.y = 1024 * rand () / (RAND_MAX + 1.0f);
    point.z = 1024 * rand () / (RAND_MAX + 1.0f);
  }

  std::cerr << "Cloud before projection: " << std::endl;
  for (const auto& point: *cloud)
    std::cerr << "    " << point.x << " "
                        << point.y << " "
                        << point.z << std::endl;

  // Create a set of planar coefficients with X=Y=0,Z=1
  // In this example, we use a plane, which has 4 parameters: normal(x,y,z), d
  // To check the coefficients of each primitive, visit headers, eg:
  // sample_consensus/sac_model_cylinder.h: point(x,y,z), direction(x,y,z), radius
  // sample_consensus/sac_model_sphere.h: center(x,y,z), radius
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
  coefficients->values.resize (4);
  coefficients->values[0] = coefficients->values[1] = 0;
  coefficients->values[2] = 1.0;
  coefficients->values[3] = 0;

  // Create the filtering object, in this case a plane
  // Other possible primitive models defined in sample_consensus/model_types.h (SAC = Sample Consensus?)
  // SACMODEL_PLANE, SACMODEL_LINE, SACMODEL_CIRCLE2D, SACMODEL_CIRCLE3D, SACMODEL_SPHERE, SACMODEL_CYLINDER, SACMODEL_CONE, ...
  pcl::ProjectInliers<pcl::PointXYZ> proj;
  proj.setModelType (pcl::SACMODEL_PLANE);
  proj.setInputCloud (cloud);
  proj.setModelCoefficients (coefficients);

  // Filter here means project on primitive (= plane, in this example)
  // Since we project the points on the XY plane, we basically set Z=0 for all the points
  proj.filter (*cloud_projected);

  std::cerr << "Cloud after projection: " << std::endl;
  for (const auto& point: *cloud_projected)
    std::cerr << "    " << point.x << " "
                        << point.y << " "
                        << point.z << std::endl;

  return (0);
}