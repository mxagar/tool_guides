/*
 * This simple file streams video from the first connected camera.
 * The window closes if a key is pressed.
 * NOTE: on Mac, you need to run it on the native Terminal
 * & have access to Camera granted for the Terminal.
 */

#include <opencv2/opencv.hpp>
#include <vector>

using namespace cv;
using namespace std;

int main(int argc, char **argv) {
    
    // Image resizing factor
    double scale = 2.0;

    // LaViola-Jones Cascade Classifier
    // There are many other classifiers in the folder: eye, body, smile...
    // All of them are trained, ready to be used
    CascadeClassifier faceCascade;
    // Load trained XML model (different location for each OS)
#if defined(WIN32) || defined(__WIN32__)
    faceCascade.load("C:\\OpenCV\\git_repos\\build\\install\etc\\haarcascades\\haarcascade_frontalface_alt.xml");
#elif __APPLE__
    faceCascade.load("/usr/local/Cellar/opencv/4.4.0_2/share/opencv4/haarcascades/haarcascade_frontalface_alt.xml");
#elif __linux__
    faceCascade.load("/usr/local/share/opencv4/haarcascades/haarcascade_frontalface_alt.xml");
#else
    faceCascade.load("/usr/local/share/opencv4/haarcascades/haarcascade_frontalface_alt.xml");
#endif

    // Video input object: 0 firsrt cam, 1 next, etc.
    VideoCapture cap(0);
    if (!cap.isOpened())
        return -1;

    while (true)
    {
        Mat frame;
        cap >> frame;
        // Convert BGR to grayscale, since Haarcascades work with gray images
        Mat grayscale;
        cvtColor(frame, grayscale, COLOR_BGR2GRAY);
        // Resize image (faster)
        resize( grayscale,
                grayscale,
                Size(grayscale.size().width / scale,
                grayscale.size().height / scale));

        // Detected faces stored in vector
        vector<Rect> faces;
        // FIND FACES!
        faceCascade.detectMultiScale(grayscale, faces, 1.1, 3, 0, Size(30, 30));
        // Loop in faces and draw them (as rectangles) on BGR frame
        for (unsigned int i = 0; i < faces.size(); ++i) {
            Scalar drawColor(255, 0, 0);
            // Define rectangle R
            Rect R(Point(cvRound(faces[i].x * scale), (cvRound(faces[i].y * scale))),
                   Point(cvRound((faces[i].x + faces[i].width - 1) * scale), (cvRound((faces[i].y + faces[i].height - 1) * scale))));
            // Draw rectangle R on frame
            rectangle(  frame,
                        R,
                        drawColor);
        }
        // Display frame image
        imshow("Webcam Frame", frame);
        if (waitKey(30) > 0)
        {
            // Wait 30ms, if any key pressed, exit
            break;
        }
    }
    return 0;
}