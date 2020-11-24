/* Simple program which applies voxelgrid filtering.
 * Pointcloud is embedded in voxel grid and for each voxel, the centroid of the points within it is delivered.
 * Look at:
 * https://pcl.readthedocs.io/projects/tutorials/en/latest/voxel_grid.html#voxelgrid
*/

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

int main (int argc, char** argv) {
  pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2 ());
  pcl::PCLPointCloud2::Ptr cloud_filtered (new pcl::PCLPointCloud2 ());

  // Fill in the cloud data
  pcl::PCDReader reader;
  // Download file:
  // https://raw.github.com/PointCloudLibrary/data/master/tutorials/table_scene_lms400.pcd
  reader.read ("../../../data/table_scene_lms400.pcd", *cloud);

  std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height 
       << " data points (" << pcl::getFieldsList (*cloud) << ")." << std::endl;

  // Filtering is performed using a voxel grid or a voxelmap
  // A voxelmap is created here with custom voxel sizes (0.01, 0.01, 0.01) m
  // In each voxel, the centroid of all points inside is computed and delivered
  // That's the filtered point cloud
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor; // sor stands for Statistical Outlier Removal
  sor.setInputCloud (cloud);
  sor.setLeafSize (0.01f, 0.01f, 0.01f);
  sor.filter (*cloud_filtered);

  std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height 
       << " data points (" << pcl::getFieldsList (*cloud_filtered) << ")." << std::endl;

  pcl::PCDWriter writer;
  writer.write ("table_scene_lms400_downsampled.pcd", *cloud_filtered, 
         Eigen::Vector4f::Zero (), Eigen::Quaternionf::Identity (), false);

  return (0);
}