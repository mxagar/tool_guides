# OpenCV Core Data Structures

This example examines the core functionalities of OpenCV as collected in
https://docs.opencv.org/4.4.0/de/d7a/tutorial_table_of_content_core.html

## `01_mat.cpp`

`cv::Mat, cv::Point2f, cv::Point3f`

Some `cv::Mat` functions not mentioned in this code:
- `cv::Mat::isContinuous()`
- `cv::Mat::channels()`
- `cv::Mat::rows()`
- `cv::Mat::cols()`


## `02_scan_images.cpp`

- `cv::Mat` image scanning is done; the exemplary application consists in reducing the pixel value range, eg [0,255] -> [0,25]
- Note that it is much faster to compute a look-up table of mapping values once and the access it for every pixel instead of computing the value at each pixel!
- `cv::getTickCount()` and `cv::getTickFrequency()`are used for computing time
- `cv::Mat` images can be single-channel or multi-channel
    - Multi-channel images contain in each row all possible channel values one after the other
    - Example: Case of color image: (Row = 0, Col = 0) -> B,G,R values one after the other (recall it's BGR, not RGB)
    - Complete rows might be stacked and stored in a large array; check if that is the case with `cv::Mat::isContinuous()`
- Four approaches are tested for reducing the pixel value range
    1. `ScanImageAndReduceC()`: second fastest function implemented in code, using typical C/C++ access operator `[]`
    2. `ScanImageAndReduceIterator()`: a `cv::MatIterator_` is used with start & end pointers
    3. `ScanImageAndReduceRandomAccess()`: (i,j) cells are accessed; inefficient for change all the image, only sensible for random pixel access
    4. `cv::LUT()` is used passing it the look-up table and the image: fastest method