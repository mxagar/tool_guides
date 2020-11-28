/* Very simple pointcloud visualization code.
 * WARNING: This example works only with the visualization module!
 * This example collects the most important visualization functionalities interesting for me.
 */

#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <string>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/integral_image_normal.h>

int main(int argc, char **argv)
{

    std::string filename;
#ifdef _WIN32
    filename = "..\\..\\..\\data\\table_scene_mug_stereo_textured.pcd";
#else
    filename = "../../data/table_scene_mug_stereo_textured.pcd";
#endif

    // Load point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile(filename, *cloud);

    // Estimate normals
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setNormalEstimationMethod(ne.AVERAGE_3D_GRADIENT);
    ne.setMaxDepthChangeFactor(0.02f);
    ne.setNormalSmoothingSize(10.0f);
    ne.setInputCloud(cloud);
    ne.compute(*normals);

    // VISUALIZATION
    pcl::visualization::PCLVisualizer viewer("PCL Viewer");

    viewer.setBackgroundColor(0.0, 0.0, 0.5); // r, g, b
    viewer.addCoordinateSystem(1.5); // size

    pcl::ModelCoefficients coeffs;
    // plane
    coeffs.values.push_back(0.0); //nx
    coeffs.values.push_back(0.0); //ny
    coeffs.values.push_back(1.0); //nz
    coeffs.values.push_back(0.0); //d
    viewer.addPlane(coeffs, "plane");
    // cone
    coeffs.values.clear();
    coeffs.values.push_back(0.3); //x
    coeffs.values.push_back(0.3); //y
    coeffs.values.push_back(0.0); //z
    coeffs.values.push_back(0.0); //nx
    coeffs.values.push_back(1.0); //ny
    coeffs.values.push_back(0.0); //nz
    coeffs.values.push_back(5.0); //alpha
    viewer.addCone(coeffs, "cone");
    // sphere
    viewer.addSphere(pcl::PointXYZ(1.0, 1.0, 1.0), 1.2, 0.5, 0.5, 0.0, "sphere"); // center, radius, r, g, b, id

    //viewer.addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud, normals); // visualize normals
    viewer.addPointCloud(cloud); // visualize points
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1); // point size
    
    while (!viewer.wasStopped())
    {
        viewer.spinOnce();
    }
    return 0;
}