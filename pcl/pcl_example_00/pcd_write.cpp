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
  /*
   Example:
   test_pcd.pcd
      # .PCD v0.7 - Point Cloud Data file format
      VERSION 0.7
      FIELDS x y z
      SIZE 4 4 4
      TYPE F F F
      COUNT 1 1 1
      WIDTH 5
      HEIGHT 1
      VIEWPOINT 0 0 0 1 0 0 0
      POINTS 5
      DATA ascii
      1.28125 577.09375 197.9375
      828.125 599.03125 491.375
      358.6875 917.4375 842.5625
      764.5 178.28125 879.53125
      727.53125 525.84375 311.28125
      ...
    Explanation:
      VERSION .7
      FIELDS x y z rgb
        dimensions/fields a point can have
        x y z
        x y z rgb
        x y z normal_x normal_y normal_z
        j1 j2 j3
      SIZE 4 4 4 4
        size in bytes of each dimension/field	
          unsigned char/char has 1 byte
          unsigned short/short has 2 bytes
          unsigned int/int/float has 4 bytes
          double has 8 bytes
      TYPE F F F F
        type of each dimension/field
        I int
        U unsigned char/int
        F float
      COUNT 1 1 1 1
        number of elements in each dimension/field
        usually 1, but with histograms, more!
        eg, VFH -> 308
      WIDTH 213
        unordered pointclouds: WIDTH N, HEIGHT 1 -> number points: N
        ordered pointclouds: WIDTH W, HEIGHT H -> number points: W*N
      HEIGHT 1
        see WIDTH
      VIEWPOINT 0 0 0 1 0 0 0
        acquisition viewpoint of dataset
        translation (tx ty tz) + quaternion (qw qx qy qz)
      POINTS 213
        see WIDTH
      DATA ascii
        ascii or binary
        after DATA, point list continues, each point ending with \n
      0.93773 0.33763 0 4.2108e+06
      0.90805 0.35641 0 4.2108e+06
      0.81915 0.32 0 4.2108e+06
      ...
  */

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