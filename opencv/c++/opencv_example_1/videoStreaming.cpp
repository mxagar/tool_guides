#include <stdio.h>
#include <iostream>
#include <opencv2/opencv.hpp>
using namespace cv;
int main(int argc, char** argv )
{

    // Video input object: 0 firsrt cam, 1 next, etc.
    VideoCapture cap(0);
    if (!cap.isOpened())
        return -1;

    Mat frame;
    while (true) {
        cap >> frame;
        imshow("Webcam Frame", frame);
        if (waitKey(30) > 0) {
            // Wait 30ms, if any key pressed, exit
            break;
        }
    }

}