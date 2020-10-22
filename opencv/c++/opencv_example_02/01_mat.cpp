/*
 * This file show-cases the core functionalities of OpenCV.
 * https://docs.opencv.org/4.4.0/d6/d6d/tutorial_mat_the_basic_image_container.html
 * NOTE: on Mac, you need to run it on the native Terminal
 * (& have access to Camera granted for the Terminal).
 */

#include <string>
#include <iostream>
#include <vector>

#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

int main(int argc, char** argv )
{

    // ::::::::: Section 1: Mat, the basic container :::::::::

    /*
    - Originally implemented in C, now C for embedded, C++ for the rest
    - cv::Mat is the most important data storage
        - it allocates/deallocates memorey automatically
        - it reuses allocated memory efficiently
        - internally 2 parts: actual array/matrix of values + header (size info, pointer, ...)
        - two Mat objects might have distict headers but share the underlying matrix/array
        - copy and passing operations are done with the header, the matrix remains often unmoved and shared!
        - references are counted: when we copy (a header), counter is increased, when be destroy one, decreased
            -> when counter is 0, underlying matrix is freed
    */

    cout << "\n-- Section 1: Mat, th ebasic container:" << endl;

#ifdef _WIN32
    string filename = "..\\..\\..\\share\\lena.png";
#else
    string filename = "../../share/lena.png";
#endif
    // All these Mats have a different headers but the same underlying matrix
    Mat A, C; // creates just the header parts
    A = imread(filename, IMREAD_COLOR); // allocate matrix
    Mat B(A); // constructor: only headers copied!
    C = A; // assignment operator: only headers copied!

    // You can create headers which refer to only a subsection of the full data
    Mat D (A, Rect(10, 10, 100, 100) ); // using a rectangle (x1, y1, x2, y2)
    Mat E = A(Range::all(), Range(1,3)); // using row and column boundaries: all rows, cols 1-3

    // Create a new copy (including the underlying matrix)
    // Changes of F & G won't affect A
    Mat F = A.clone();
    Mat G;
    A.copyTo(G);

    // ::::::::: Section 2: Storing methods :::::::::

    /*
    Color spaces: 3-4 channels
    - RGB, RGBA, A = alpha = transparency
    - BGR: DEFAULT OpenCV!
    - HSV/HSL: hue, saturation, value/luminance -> very useful for color analysis
    - YCrCb, used by JPEG
    - CIE L*a*b*: perceptually uniform color space, for measuring distances between colors
    Values in pixels:
    - usually char: 1 byte = 8 bit = 0-255
    - but we can also have more memory expensive values: 4 byte = 32 bit, 8 byte = 64 bit
    */

    cout << "\n-- Section 2: Storing methods:" << endl;

    // ::::::::: Section 3: Creating a Mat object explicitly :::::::::

    cout << "\n-- Section 3: Creating a Mat object explicitly:" << endl;

    // Constructor: rows, columns, pixel value type, initialization value
    // pixel value type: constants built like this: CV_[bits per item][un/signed][type prefix]C[channel number]
    Mat M(2,2, CV_8UC3, Scalar(0,0,255));
    // Output - only for 2D matrices
    cout << "M = " << endl << " " << M << endl << endl;

    // 3D Matrix with width 2 in each dimension + 1 channel CV_8UC(1)
    // CV_8UC(x), x can be custom, default constants/macros: CV_8UC1, CV_8UC2, CV_8UC3, CV_8UC4
    int size[3] = {2,2,2};
    Mat L(3, size, CV_8UC(1), Scalar::all(0));

    // Existing matrix data memory of M re-used, no initialization possible
    M.create(4,4, CV_8UC(2));
    cout << "M = "<< endl << " "  << M << endl << endl;

    // Matlab style initializers: eye, ones, zeros
    Mat Y = Mat::eye(4, 4, CV_64F);
    cout << "Y = " << endl << " " << Y << endl << endl;
    Mat O = Mat::ones(2, 2, CV_32F);
    cout << "O = " << endl << " " << O << endl << endl;
    Mat Z = Mat::zeros(3,3, CV_8UC1);
    cout << "Z = " << endl << " " << Z << endl << endl;

    // Fill in with random values
    Mat R = Mat(3, 2, CV_8UC3);
    randu(R, Scalar::all(0), Scalar::all(255));
    cout << "R = " << endl << " " << R << endl << endl;

    // ::::::::: Section 4: Output formatting :::::::::

    cout << "\n-- Section 4: Output formatting:" << endl;

    cout << "R (default) = " << endl << R << endl << endl;
    cout << "R (python)  = " << endl << format(R, Formatter::FMT_PYTHON) << endl << endl;
    cout << "R (csv) = " << endl << format(R, Formatter::FMT_CSV) << endl << endl;
    cout << "R (numpy) = " << endl << format(R, Formatter::FMT_NUMPY ) << endl << endl;

    // ::::::::: Section 5: Other types & output :::::::::

    cout << "\n-- Section 5: Other types & output:" << endl;

    Point2f P(5, 1);
    cout << "Point (2D) = " << endl << P << endl << endl;

    Point3f P3f(2, 6, 7);
    cout << "Point (3D) = " << endl << P3f << endl << endl;

    vector<float> v;
    v.push_back((float)CV_PI);
    v.push_back(2);
    v.push_back(3.01f);
    cout << "Vector of floats via Mat = " << endl << Mat(v) << endl << endl;

    vector<Point2f> vPoints(20);
    for (size_t i = 0; i < vPoints.size(); ++i)
        vPoints[i] = Point2f((float)(i * 5), (float)(i % 7));
    cout << "A vector of 2D Points = " << endl << vPoints << endl << endl;

    return 0;

}