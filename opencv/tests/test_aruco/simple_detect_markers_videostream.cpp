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

    // Video input object: 0 firsrt cam, 1 next, etc.
    cv::VideoCapture inputVideo;
    inputVideo.open(0);
    if (!inputVideo.isOpened())
        return -1;

    // The DICTIONARY
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);

    while (inputVideo.grab()) {

        // Grab frame
        cv::Mat frameIn, frameOut;
        inputVideo.retrieve(frameIn);
        frameIn.copyTo(frameOut);

        // Loop parameters: marker corners & IDs
        std::vector<int> markerIds;
        std::vector<std::vector<cv::Point2f>> markerCorners;

        // The call to detect markers with all params
        cv::aruco::detectMarkers(	frameOut,
                                    dictionary,
                                    markerCorners,
                                    markerIds);

        // Results:
        // - markerCorners: marker 2d points clockwise, starting from top-left
        // - markerIds
        // - markerCorners.size() == markerIds.size()

        // Draw result
        if (markerIds.size() > 0)
            cv::aruco::drawDetectedMarkers(frameOut, markerCorners, markerIds);

        imshow("Webcam Frame with Markers", frameOut);

        if (waitKey(30) > 0) {
            // Wait 30ms, if any key pressed, exit
            break;
        }
    }
    return 0;

}