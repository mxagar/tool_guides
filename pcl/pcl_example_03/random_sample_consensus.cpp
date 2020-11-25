/* Simple program in which a pointcloud containing inliers & putliers of a model (plane or sphere) is segmented using RANSAC
 * Basically, the inliers that fit the model (plane / sphere) are detected (and visualized)
 * Example of usage:
 *    ./random_sample_consensus
 *        plane points + outliers created, everything visualized
 *    ./random_sample_consensus -f
 *        plane points + outliers created, only fitted inliers visualized
 *    ./random_sample_consensus -s
 *        sphere points + outliers created, everything visualized
 *    ./random_sample_consensus -sf
 *        sphere points + outliers created, only fitted inliers visualized 
 * Look at:
 * https://pcl.readthedocs.io/projects/tutorials/en/latest/random_sample_consensus.html#random-sample-consensus
*/

#include <iostream>
#include <thread>

#include <pcl/console/parse.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>

// If PCL was compiled with the visualization module, uncommet this!
#ifndef VISUALIZE
//#define VISUALIZE
#endif

#ifdef VISUALIZE
#include <pcl/visualization/pcl_visualizer.h>
#endif

using namespace std::chrono_literals;

#ifdef VISUALIZE
pcl::visualization::PCLVisualizer::Ptr
simpleVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  viewer->addCoordinateSystem (1.0, "global");
  viewer->initCameraParameters ();
  return (viewer);
}
#endif

int main(int argc, char** argv) {
  
  // Initialize PointClouds:
  // - cloud is filled with data points
  // - final will be filled with the model inliers computed from cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr final (new pcl::PointCloud<pcl::PointXYZ>);

  // Populate our PointCloud with points: random + sphere (-f, -sf) / plane (-f)
  cloud->width    = 500;
  cloud->height   = 1;
  cloud->is_dense = false;
  cloud->points.resize (cloud->width * cloud->height);
  for (pcl::index_t i = 0; i < cloud->size (); ++i)
  {
    if (pcl::console::find_argument (argc, argv, "-s") >= 0 || pcl::console::find_argument (argc, argv, "-sf") >= 0)
    {
      // -s: sphere, -sf: sphere (but visualize only inliers)
      (*cloud)[i].x = 1024 * rand () / (RAND_MAX + 1.0);
      (*cloud)[i].y = 1024 * rand () / (RAND_MAX + 1.0);
      if (i % 5 == 0)
        // 1/5 of all points are completely random
        (*cloud)[i].z = 1024 * rand () / (RAND_MAX + 1.0);
      else if(i % 2 == 0)
        // rest of points belong to a sphere in origin with radius 1.0
        (*cloud)[i].z =  sqrt( 1 - ((*cloud)[i].x * (*cloud)[i].x)
                                      - ((*cloud)[i].y * (*cloud)[i].y));
      else
        // rest of points belong to a sphere in origin with radius 1.0
        (*cloud)[i].z =  - sqrt( 1 - ((*cloud)[i].x * (*cloud)[i].x)
                                        - ((*cloud)[i].y * (*cloud)[i].y));
    }
    else
    {
      // no argument or -f: plane
      (*cloud)[i].x = 1024 * rand () / (RAND_MAX + 1.0);
      (*cloud)[i].y = 1024 * rand () / (RAND_MAX + 1.0);
      if( i % 2 == 0)
        // 1/2 of the points are completely random
        (*cloud)[i].z = 1024 * rand () / (RAND_MAX + 1.0);
      else
        // rest of the points belong to the plane x+y+z=0
        (*cloud)[i].z = -1 * ((*cloud)[i].x + (*cloud)[i].y);
    }
  }

  // Indices of inliers stored in a vector of ints
  std::vector<int> inliers;

  // Define the appropriated model: Sphere / Plane
  // There are many more models (see model_types.h)
  // Sphere, Plane, Line, Circle2D, Circle3D, Cylinder, Cone, ...
  pcl::SampleConsensusModelSphere<pcl::PointXYZ>::Ptr
    model_s(new pcl::SampleConsensusModelSphere<pcl::PointXYZ> (cloud));
  pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr
    model_p (new pcl::SampleConsensusModelPlane<pcl::PointXYZ> (cloud));
  // Create RandomSampleConsensus (RANSAC) object, pass model & compute the model
  if(pcl::console::find_argument (argc, argv, "-f") >= 0)
  {
    // Plane (compute inliers)
    pcl::RandomSampleConsensus<pcl::PointXYZ> ransac (model_p);
    ransac.setDistanceThreshold (.01);
    ransac.computeModel();
    // Indices of cloud that are inliers
    ransac.getInliers(inliers);
  }
  else if (pcl::console::find_argument (argc, argv, "-sf") >= 0 )
  {
    // Sphere (compute inliers; if -s, no inliers computed)
    pcl::RandomSampleConsensus<pcl::PointXYZ> ransac (model_s);
    ransac.setDistanceThreshold (.01);
    ransac.computeModel();
    // Indices of cloud that are inliers
    // NOTE: I don't know why, on Windows I get only 4 inliers...?
    ransac.getInliers(inliers);
  }

  // Create a new pointcloud (final) which contains the model inliers from cloud
  // Indices are passed: inliers
  pcl::copyPointCloud (*cloud, inliers, *final);

  std::cout << "Original point cloud: \n" << *cloud << std::endl;
  std::cout << "Fitted point cloud: \n" << *final << std::endl;

#ifdef VISUALIZE
  // creates the visualization object and adds either our original cloud or all of the inliers
  // depending on the command line arguments specified.
  pcl::visualization::PCLVisualizer::Ptr viewer;
  if (pcl::console::find_argument (argc, argv, "-f") >= 0 || pcl::console::find_argument (argc, argv, "-sf") >= 0)
    viewer = simpleVis(final);
  else
    viewer = simpleVis(cloud);
  while (!viewer->wasStopped ())
  {
    viewer->spinOnce (100);
    std::this_thread::sleep_for(100ms);
  }
#endif

  return 0;
 }