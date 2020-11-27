/* Create a 2D convex / concave hull (3D points that enclose the hull) of a planar point cloud.
 * A simple 2D hull polygon (concave or convex) for a set of points supported by a plane.
 * Look at:
 * https://pcl.readthedocs.io/projects/tutorials/en/latest/hull_2d.html#hull-2d
*/

#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/surface/convex_hull.h>

int main(int argc, char **argv)
{
    bool convex = true;
    if (argc > 1) {
        if (strcmp(argv[1], "-concave") == 0) {
            convex = false;
        } // else: -convex
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>),
        cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>),
        cloud_projected(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PCDReader reader;

    // Download from:
    // https://raw.github.com/PointCloudLibrary/data/master/tutorials/table_scene_mug_stereo_textured.pcd
    // scene is a table with some other artefacts
    reader.read("../../data/table_scene_mug_stereo_textured.pcd", *cloud);
    // Build a filter to remove spurious NaNs and remain only with Z range [0,1.1]
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0, 1.1);
    pass.filter(*cloud_filtered);
    std::cerr << "PointCloud after filtering has: "
              << cloud_filtered->size() << " data points." << std::endl;

    // Segment the points that are on a plane
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // Optional
    seg.setOptimizeCoefficients(true);
    // Mandatory
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.01);
    // Filtering
    seg.setInputCloud(cloud_filtered);
    seg.segment(*inliers, *coefficients);
    std::cerr << "PointCloud after segmentation has: "
              << inliers->indices.size() << " inliers." << std::endl;

    // Project the model inliers: project points to the plane
    pcl::ProjectInliers<pcl::PointXYZ> proj;
    proj.setModelType(pcl::SACMODEL_PLANE);
    // proj.setIndices (inliers);
    proj.setInputCloud(cloud_filtered);
    proj.setModelCoefficients(coefficients);
    proj.filter(*cloud_projected);
    std::cerr << "PointCloud after projection has: "
              << cloud_projected->size() << " data points." << std::endl;

    // Create a 2D CONVEX / CONCAVE HULL: select only 3D points that enclose that hull
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull(new pcl::PointCloud<pcl::PointXYZ>);
    if (convex) {
        // Convex Hull
        // Create a Convex Hull representation of the projected inliers
        pcl::ConvexHull<pcl::PointXYZ> chull;
        chull.setInputCloud(cloud_projected);
        chull.reconstruct(*cloud_hull);

        std::cerr << "Convex hull has: " << cloud_hull->size()
                  << " data points." << std::endl;

    } else {
        // Concave Hull
        // Create a Concave Hull representation of the projected inliers
        pcl::ConcaveHull<pcl::PointXYZ> chull;
        chull.setInputCloud(cloud_projected);
        chull.setAlpha(0.1);
        chull.reconstruct(*cloud_hull);

        std::cerr << "Concave hull has: " << cloud_hull->size()
                  << " data points." << std::endl;

    }

    pcl::PCDWriter writer;
    writer.write("table_scene_mug_stereo_textured_hull.pcd", *cloud_hull, false);

    return (0);
}