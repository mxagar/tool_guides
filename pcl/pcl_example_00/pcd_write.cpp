#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

int main (int argc, char** argv) {

  // Basic Point Cloud Data Structure PCD, defined in point_cloud.h
  pcl::PointCloud<pcl::PointXYZ> cloud;
  // PCDs can be:
  // - organized: data in image matrices with width x height dimensions (ie, pixels), faster processing because neighbors known
  // - unorganized: random list of 3D points; slower processing
  // A PCD is projectable when organized pixel indices (u,v) can be projected into 3D with th epinhol ecamera model
  // u = f*(x/z), v = f*(y/z)
  // The point type (PointT) must be passed to the PCD constructor
  // There are many point types inherited from PointT defined in point_types.h
  // PointXYZ, PointXYZRGB, PointNormal, PointXYZRGBNormal, PointXYZHSV, ...
  // See:
  // https://pcl.readthedocs.io/projects/tutorials/en/latest/adding_custom_ptype.html#adding-custom-ptype

  // PCD contains these data fields:
  cloud.width    = 5; // rows (organized PCDs) or number of points (unorganized PCDs)
  cloud.height   = 1; // columns (organized PCDs) or 1 == unorganized PCDs
  cloud.is_dense = false; // true == values of points finite numbers, else: could contains NaNs
  // Most important field: points; type is passed to PCD constructor
  // points is a std::vector of the selected point type!
  cloud.points.resize (cloud.width * cloud.height);
  
  // Additional PCD fields/member variables:
  // cloud.sensor_origin_ -> Eigen::Vector4f (optional): acquisition pose translation (origin)
  // cloud.sensor_orientation_ -> Eigen::Quaternionf (optional): acquisition pose orientation

  // Additional PCD methods:
  // cloud.isOrganized()
  // cloud.assign()
  // cloud.push_back()
  // cloud.size()
  // cloud.resize()

  for (auto& point: cloud)
  {
    point.x = 1024 * rand () / (RAND_MAX + 1.0f);
    point.y = 1024 * rand () / (RAND_MAX + 1.0f);
    point.z = 1024 * rand () / (RAND_MAX + 1.0f);
  }

  std::string filename = "test_pcd.pcd";
  pcl::io::savePCDFileASCII (filename, cloud);
  std::cerr << "Saved " << cloud.size () << " data points to test_pcd.pcd." << std::endl;
  // test_pcd.pcd
  //    # .PCD v0.7 - Point Cloud Data file format
  //    VERSION 0.7
  //    FIELDS x y z
  //    SIZE 4 4 4
  //    TYPE F F F
  //    COUNT 1 1 1
  //    WIDTH 5
  //    HEIGHT 1
  //    VIEWPOINT 0 0 0 1 0 0 0
  //    POINTS 5
  //    DATA ascii
  //    1.28125 577.09375 197.9375
  //    828.125 599.03125 491.375
  //    358.6875 917.4375 842.5625
  //    764.5 178.28125 879.53125
  //    727.53125 525.84375 311.28125

  std::cout << "Saved PCD: " << std::endl;
  for (const auto& point: cloud)
    std::cerr << "    " << point.x << " " << point.y << " " << point.z << std::endl;

  pcl::PointCloud<pcl::PointXYZ> new_cloud;
  pcl::io::loadPCDFile(filename, new_cloud);

  std::cout << "\nRead PCD: " << std::endl;
  for (const auto& point: cloud)
    std::cerr << "    " << point.x << " " << point.y << " " << point.z << std::endl;

  return (0);
}