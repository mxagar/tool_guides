/* Very simple range image creation from a point cloud.
 * This tutorial demonstrates how to create a range image from a point cloud and a given sensor position.
 */

#include <pcl/range_image/range_image.h>

int main(int argc, char **argv)
{
    pcl::PointCloud<pcl::PointXYZ> pointCloud;

    // Generate the pointcloud data
    for (float y = -0.5f; y <= 0.5f; y += 0.01f)
    {
        for (float z = -0.5f; z <= 0.5f; z += 0.01f)
        {
            // x + y = 2 plane
            pcl::PointXYZ point;
            point.x = 2.0f - y;
            point.y = y;
            point.z = z;
            pointCloud.points.push_back(point);
        }
    }
    // We have an unordered pointcloud!
    pointCloud.width = pointCloud.size();
    pointCloud.height = 1;

    // Parameters necessary for the range image
    // - angularResolution: beams represented by neighboring pixels differ by much degrees?
    // - maxAngleWidth & maxAngleHeight: 360 & 180 means everything is observed by the sensor
    //      we can always leave it like that, since image will be cropped to where something (points) appear - but if we reduce 360, faster calculation
    // - sensorPose
    // - coordinate_frame: coordinate system to project image, usually CAMERA_FRAME = X right, Y downwards, Z forwards
    //      alternative: LASER_FRAME, with x facing forward, y to the left and z upwards.
    // - noiseLevel: 0 = normal Z-buffer; 0.05 = all points with a maximum distance of 5cm to the closest point are used to calculate the range
    // - minRange: if > 0, all points that are closer will be ignored
    // - borderSize; if > 0, border of unobserved points around the image when cropping it is left
    // We want to create a range image from the above point cloud, with a 1deg angular resolution
    float angularResolution = (float)(1.0f * (M_PI / 180.0f)); //   1.0 degree in radians
    float maxAngleWidth = (float)(360.0f * (M_PI / 180.0f));   // 360.0 degree in radians
    float maxAngleHeight = (float)(180.0f * (M_PI / 180.0f));  // 180.0 degree in radians
    Eigen::Affine3f sensorPose = (Eigen::Affine3f)Eigen::Translation3f(0.0f, 0.0f, 0.0f); // origin, with roll=pitch=yaw=0.0
    pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME; // X right, Y downwards, Z forwards
    float noiseLevel = 0.00;
    float minRange = 0.0f;
    int borderSize = 1;

    // Create range image with all parameters
    pcl::RangeImage rangeImage;
    rangeImage.createFromPointCloud(pointCloud, angularResolution, maxAngleWidth, maxAngleHeight,
                                    sensorPose, coordinate_frame, noiseLevel, minRange, borderSize);

    // Notes:
    // - pcl::RangeImage is derived from pcl::PointCloud
    // - points of pcl::PointCloud have 3 members: x, y, z, range
    // - there are 3 types of points
    //      1. Valid points: range > 0
    //      2. Unobserved points: x=y=z=NAN, range=-INFINITY
    //      3. Far range points: x=y=z=NAN, range=INFINITY

    // Output operator is implemented
    std::cout << rangeImage << "\n";
    
}