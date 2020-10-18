cmake_minimum_required(VERSION 3.0)
project(VideoStreaming)

# Set OpenCV_DIR to the path that contains
# either 'OpenCVConfig.cmake' or 'FindOpenCV.cmake'
if(WIN32)
    # After regular installation of binaries from opencv.org
    #set(OpenCV_DIR C:/OpenCV/OpenCV-4.4.0/opencv/build)
    # After installation from source + contrib
    set(OpenCV_DIR C:/OpenCV/git_repos/build)
elseif(APPLE)
    # After 'brew install opencv'
    set(OpenCV_DIR /usr/local/Cellar/opencv/4.4.0_2/lib/cmake/opencv4)
elseif(LINUX)
    # After compilation from source
    set(OpenCV_DIR /usr/local/lib/cmake/opencv4)
endif(WIN32)
find_package(OpenCV REQUIRED)
# if found, automatically defined variables
# OpenCV_FOUND
# OpenCV_INCLUDE_DIRS or OpenCV_INCLUDES
# OpenCV_LIBRARIES or OpenCV_LIBS
# OpenCV_DEFINITIONS

include_directories(${OpenCV_INCLUDE_DIRS})
add_executable(videoStreaming videoStreaming.cpp)
add_executable(faceDetection faceDetection.cpp)
target_link_libraries(videoStreaming ${OpenCV_LIBS})
target_link_libraries(faceDetection ${OpenCV_LIBS})