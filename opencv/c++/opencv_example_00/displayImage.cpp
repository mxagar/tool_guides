/*
 * This simple application opens an image with OpenCV.
 */

#include <stdio.h>
#include <iostream>
#include <opencv2/opencv.hpp>
using namespace cv;
int main(int argc, char** argv )
{
    if ( argc != 2 )
    {
        //printf("usage: displayImage <Image_Path>\n");
        std::cout << "usage: displayImage <Image_Path>" << std::endl;
        return -1;
    }
    Mat image;
    image = imread( argv[1], 1 );
    if ( !image.data )
    {
        //printf("No image data \n");
        std::cout << "No image data" << std::endl;
        return -1;
    }
    namedWindow("Display Image", WINDOW_AUTOSIZE );
    imshow("Display Image", image);
    waitKey(0);
    return 0;
}