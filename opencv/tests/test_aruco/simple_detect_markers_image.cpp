/* Simple program that detects Aruco markers given an image.
 * The aruco module from OpenCVis used.
 * Look at:
 * https://docs.opencv.org/master/d5/dae/tutorial_aruco_detection.html
*/

#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <iostream>
#include <string>

using namespace std;
using namespace cv;

int main(int argc, char *argv[]) {

    string filename;
    if (argc > 1) {
        filename = argv[1];
    } else {
#ifdef _WIN32
        filename = "..\\..\\data\\markers_image.jpg";
#else
        filename = "../../data/markers_image.jpg";
#endif
    }

    // The image
    cv::Mat inputImage;
    inputImage = imread(filename, 1);

    // The parameters
    std::vector<int> markerIds;
    std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
    cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
    // IMPORTANT NOTE: the input image should have markers from this ditcionary!
    // We can create markers with create_markers.cpp
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);

    // The call to detect markers with all params
    // parameters & rejectedCandidates are optional
    cv::aruco::detectMarkers(	inputImage,
                                dictionary,
                                markerCorners,
                                markerIds,
                                parameters,
                                rejectedCandidates);

    // Results:
    // - markerCorners: marker 2d points clockwise, starting from top-left
    // - markerIds
    // - markerCorners.size() == markerIds.size()

    // Draw result
    cv::Mat outputImage = inputImage.clone();
    cv::aruco::drawDetectedMarkers(outputImage, markerCorners, markerIds);

    namedWindow("Display Image with Markers", WINDOW_NORMAL);
    imshow("Display Image with Markers", outputImage);
    waitKey(0);
    return 0;

}