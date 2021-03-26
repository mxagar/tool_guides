/*
 * This file show-cases the core functionalities of OpenCV.
 * https://docs.opencv.org/4.4.0/d7/d37/tutorial_mat_mask_operations.html
 * The code was taken from the OpenCV tutorials and slightly modified.
 * NOTE: on Mac, you need to run it on the native Terminal
 */

#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>
using namespace std;
using namespace cv;
static void help(char *progName)
{
    cout << endl
         << "This program shows how to filter images with mask: the write it yourself and the"
         << "filter2d way. " << endl
         << "Usage:" << endl
         << progName << " [image_path -- default lena.jpg] [G -- grayscale] " << endl
         << endl;
}
void Sharpen(const Mat &myImage, Mat &Result);
int main(int argc, char *argv[])
{
    help(argv[0]);
    const char *filename = argc >= 2 ? argv[1] : "lena.jpg";
    Mat src, dst0, dst1;
    if (argc >= 3 && !strcmp("G", argv[2]))
        src = imread(samples::findFile(filename), IMREAD_GRAYSCALE);
    else
        src = imread(samples::findFile(filename), IMREAD_COLOR);
    if (src.empty())
    {
        cerr << "Can't open image [" << filename << "]" << endl;
        return EXIT_FAILURE;
    }
    namedWindow("Input", WINDOW_AUTOSIZE);
    namedWindow("Output", WINDOW_AUTOSIZE);
    imshow("Input", src);

    // Option 1: manually
    double t = (double)getTickCount();
    Sharpen(src, dst0);
    t = ((double)getTickCount() - t) / getTickFrequency();
    cout << "Hand written function time passed in seconds: " << t << endl;
    imshow("Output", dst0);
    waitKey();
    
    // Option 2: using cv::filter2D()
    // On my Mac, filter2D is approx. 9x faster
    Mat kernel = (Mat_<char>(3, 3) << 0, -1, 0,
                  -1, 5, -1,
                  0, -1, 0);
    t = (double)getTickCount();
    // cv::filter2D() applies any convolutional kernel passed as a cv::Mat
    // it has more arguments: center of kernel, treatment of border pixels, etc.
    filter2D(src, dst1, src.depth(), kernel);
    t = ((double)getTickCount() - t) / getTickFrequency();
    cout << "Built-in filter2D time passed in seconds: " << t << endl;
    imshow("Output", dst1);
    waitKey();

    return EXIT_SUCCESS;
}

// Manual convolution: 0, -1, 0; - 1, 5, -1; 0, -1, 0
// This is 9x slower than filter2D on my Mac
// But you learn how to efficiently iterate and apply neighboring operations pixel-wise on an image 
void
Sharpen(const Mat &myImage, Mat &Result)
{
    CV_Assert(myImage.depth() == CV_8U); // accept only uchar images: unsigned char
    const int nChannels = myImage.channels();
    Result.create(myImage.size(), myImage.type());
    for (int j = 1; j < myImage.rows - 1; ++j)
    {
        const uchar *previous = myImage.ptr<uchar>(j - 1);
        const uchar *current = myImage.ptr<uchar>(j);
        const uchar *next = myImage.ptr<uchar>(j + 1);
        uchar *output = Result.ptr<uchar>(j);
        for (int i = nChannels; i < nChannels * (myImage.cols - 1); ++i)
        {
            *output++ = saturate_cast<uchar>(5 * current[i] - current[i - nChannels] - current[i + nChannels] - previous[i] - next[i]);
        }
    }
    // We cannot straightforwardly apply the convolution on the border pixels
    // Naive solution: do not apply convolution in them and, at the end, set them to value 0
    Result.row(0).setTo(Scalar(0));
    Result.row(Result.rows - 1).setTo(Scalar(0));
    Result.col(0).setTo(Scalar(0));
    Result.col(Result.cols - 1).setTo(Scalar(0));
}