/* Simple program that computes the normals of an ordered pointcloud created from depth images.
 * Integral images or summed-area tables are used internally for computing the normals.
 * Look at:
 * https://pcl.readthedocs.io/projects/tutorials/en/latest/normal_estimation_using_integral_images.html#normal-estimation-using-integral-images
 * This example was created from the normal_estimation_using_integral_images.cpp code.
*/

// If PCL was compiled with the visualization module, uncommet this!
#ifndef VISUALIZE
#define VISUALIZE
#endif

#ifdef VISUALIZE
#include <pcl/visualization/cloud_viewer.h>
#endif
#include <iostream>
#include <string>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/integral_image_normal.h>
    
int main (int argc, char** argv) {

    // Parse the arguments or define the filename
    std::string filename;
    if (argc > 1) {
        filename = std::string(argv[1]);
    } else {
#ifdef _WIN32
    filename = "..\\..\\..\\data\\table_scene_mug_stereo_textured.pcd";
#else
    filename = "../../data/table_scene_mug_stereo_textured.pcd";
#endif
    }

    // load point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);    
    pcl::io::loadPCDFile (filename, *cloud);
    
    // estimate normals
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);

    pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    // Available normal estmation methods:
    // enum NormalEstimationMethod {COVARIANCE_MATRIX, AVERAGE_3D_GRADIENT, AVERAGE_DEPTH_CHANGE};
    // With each method, integral images (ie, summed area tables) are computed and used,
    // ie, images in which each pixel contains the sum  of all the pior values
    // - COVARIANCE_MATRIX creates 9 integral images to compute the normal for a specific point from the covariance matrix of its local neighborhood
    // - AVERAGE_3D_GRADIENT creates 6 integral images to compute smoothed versions of horizontal and vertical 3D gradients and computes the normals using the cross-product between these two gradients
    // - AVERAGE_DEPTH_CHANGE creates only a single integral image and computes the normals from the average depth changes
    ne.setNormalEstimationMethod (ne.AVERAGE_3D_GRADIENT);
    ne.setMaxDepthChangeFactor(0.02f);
    ne.setNormalSmoothingSize(10.0f);
    ne.setInputCloud(cloud);
    ne.compute(*normals);

    // visualize normals
#ifdef VISUALIZE
    pcl::visualization::PCLVisualizer viewer("PCL Viewer");
    viewer.setBackgroundColor (0.0, 0.0, 0.5);
    viewer.addPointCloudNormals<pcl::PointXYZ,pcl::Normal>(cloud, normals);
    
    while (!viewer.wasStopped ())
    {
      viewer.spinOnce ();
    }
#endif

    return 0;
}