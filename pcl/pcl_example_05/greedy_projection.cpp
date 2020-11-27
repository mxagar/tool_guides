/* Greedy surface triangulation algorithm on a PointCloud with normals, to obtain a triangle mesh based on projections of the local neighborhoods.
 * Triangulation is performed locally, by projecting the local neighborhood of a point along the point’s normal, and connecting unconnected points.
 * Look at:
 * https://pcl.readthedocs.io/projects/tutorials/en/latest/greedy_projection.html#greedy-triangulation
*/

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>

int main(int argc, char **argv)
{
    // Load input file into a PointCloud<T> with an appropriate type
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PCLPointCloud2 cloud_blob;
    // Download from
    // https://raw.githubusercontent.com/PointCloudLibrary/pcl/master/test/bun0.pcd
    pcl::io::loadPCDFile("../../data/bun0.pcd", cloud_blob);
    pcl::fromPCLPointCloud2(cloud_blob, *cloud);
    //* the data should be available in cloud

    // Normal estimation*
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud);
    n.setInputCloud(cloud);
    n.setSearchMethod(tree);
    n.setKSearch(20);
    n.compute(*normals);
    //* normals should not contain the point normals + surface curvatures

    // Concatenate the XYZ and normal fields*
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
    pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);
    //* cloud_with_normals = cloud + normals

    // Create search tree*
    pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
    tree2->setInputCloud(cloud_with_normals);

    // Initialize objects
    pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
    pcl::PolygonMesh triangles;

    // Set the maximum distance between connected points (maximum edge length)
    gp3.setSearchRadius(0.025);

    // Set typical values for the parameters
    // See description at
    // https://pcl.readthedocs.io/projects/tutorials/en/latest/greedy_projection.html#greedy-triangulation
    gp3.setMu(2.5);
    gp3.setMaximumNearestNeighbors(100);
    gp3.setMaximumSurfaceAngle(M_PI / 4); // 45 degrees
    gp3.setMinimumAngle(M_PI / 18);       // 10 degrees
    gp3.setMaximumAngle(2 * M_PI / 3);    // 120 degrees
    gp3.setNormalConsistency(false);

    // Get result
    gp3.setInputCloud(cloud_with_normals);
    gp3.setSearchMethod(tree2);
    gp3.reconstruct(triangles);

    // Additional vertex information
    std::vector<int> parts = gp3.getPartIDs();
    // For each point, states can be gp3.FREE, gp3.BOUNDARY or gp3.COMPLETED
    std::vector<int> states = gp3.getPointStates();

    //pcl::PCDWriter writer;
    //writer.write("mesh.pcd", triangles);
    pcl::io::saveVTKFile("mesh.vtk", triangles);

    // Finish
    return (0);
}