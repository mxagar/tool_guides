/*
 * This simple file streams video from the first connected camera.
 * The window closes if a key is pressed.
 * NOTE: on Mac, you need to run it on the native Terminal
 * & have access to Camera granted for the Terminal.
 */

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
    return 0;
}