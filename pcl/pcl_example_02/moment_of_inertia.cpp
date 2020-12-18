/* Simple program with which descriptors based on moments of inertia extracted: 
 * eccentricity, center of mass, AABB, (non-optimal) OBB, inertia tensor values, eigen system values, etc.
 * First, the covariance matrix of the pointcloud is computed and its eigenvealues and vectors are extracted:
 * major vector is X (normalized), then Y, and the minor vector is Z.
 * Then, the major axis (X) is rotated around the pthers to achieve rotational invariance (???)
 * - for each rotation, the moments and eccentricity are computed
 * - eccentricity is computed with the points projected on the plane with normal the current rotated vector
 * WATCH OUT: visualization module is necessary if you want to display the results: AABB, OBB, eigenvectors
 * Look at:
 * https://pcl.readthedocs.io/en/latest/moment_of_inertia.html#moment-of-inertia
*/

// If PCL was compiled with the visualization module, uncommet this!
#ifndef VISUALIZE
#define VISUALIZE
#endif

#ifdef VISUALIZE
#include <pcl/visualization/cloud_viewer.h>
#endif

#include <vector>
#include <thread>

#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
    std::string filename;
    if (argc != 2) {
        //return (0);
        // Use default example file
#ifdef _WIN32
        filename = "../../../data/min_cut_segmentation_tutorial.pcd";
#else
        filename = "../../data/min_cut_segmentation_tutorial.pcd";
#endif
    } else {
        filename = argv[1];
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    if (pcl::io::loadPCDFile(filename, *cloud) == -1)
        return (-1);

    // Create feature estimator, set cloud, and compute features
    pcl::MomentOfInertiaEstimation<pcl::PointXYZ> feature_extractor;
    feature_extractor.setInputCloud(cloud);
    feature_extractor.compute();

    // Extract features: AABB, OBB, CoM, EigenSystem
    std::vector<float> moment_of_inertia;
    std::vector<float> eccentricity;
    pcl::PointXYZ min_point_AABB;
    pcl::PointXYZ max_point_AABB;
    pcl::PointXYZ min_point_OBB;
    pcl::PointXYZ max_point_OBB;
    pcl::PointXYZ position_OBB;
    Eigen::Matrix3f rotational_matrix_OBB;
    float major_value, middle_value, minor_value;
    Eigen::Vector3f major_vector, middle_vector, minor_vector;
    Eigen::Vector3f mass_center;

    feature_extractor.getMomentOfInertia(moment_of_inertia);
    feature_extractor.getEccentricity(eccentricity);
    feature_extractor.getAABB(min_point_AABB, max_point_AABB);
    feature_extractor.getOBB(min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);
    feature_extractor.getEigenValues(major_value, middle_value, minor_value);
    feature_extractor.getEigenVectors(major_vector, middle_vector, minor_vector);
    feature_extractor.getMassCenter(mass_center);

#ifdef VISUALIZE

    // Extracted features are visualized with
    // addLine() & addCube()

    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);
    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();
    viewer->addPointCloud<pcl::PointXYZ>(cloud, "sample cloud");
    viewer->addCube(min_point_AABB.x, max_point_AABB.x, min_point_AABB.y, max_point_AABB.y, min_point_AABB.z, max_point_AABB.z, 1.0, 1.0, 0.0, "AABB");
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "AABB");

    Eigen::Vector3f position(position_OBB.x, position_OBB.y, position_OBB.z);
    Eigen::Quaternionf quat(rotational_matrix_OBB);
    viewer->addCube(position, quat, max_point_OBB.x - min_point_OBB.x, max_point_OBB.y - min_point_OBB.y, max_point_OBB.z - min_point_OBB.z, "OBB");
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "OBB");

    pcl::PointXYZ center(mass_center(0), mass_center(1), mass_center(2));
    pcl::PointXYZ x_axis(major_vector(0) + mass_center(0), major_vector(1) + mass_center(1), major_vector(2) + mass_center(2));
    pcl::PointXYZ y_axis(middle_vector(0) + mass_center(0), middle_vector(1) + mass_center(1), middle_vector(2) + mass_center(2));
    pcl::PointXYZ z_axis(minor_vector(0) + mass_center(0), minor_vector(1) + mass_center(1), minor_vector(2) + mass_center(2));
    viewer->addLine(center, x_axis, 1.0f, 0.0f, 0.0f, "major eigen vector");
    viewer->addLine(center, y_axis, 0.0f, 1.0f, 0.0f, "middle eigen vector");
    viewer->addLine(center, z_axis, 0.0f, 0.0f, 1.0f, "minor eigen vector");

    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
        std::this_thread::sleep_for(100ms);
    }
#endif /* VISUALIZE */

    return (0);
}