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
    # Update: now, the found version is another... 
    # include = /usr/local/Cellar/opencv/4.5.0_5/include/opencv4
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
# Print some message showing some of them
message(STATUS "OpenCV library status:")
message(STATUS "    config: ${OpenCV_DIR}")
message(STATUS "    version: ${OpenCV_VERSION}")
message(STATUS "    libraries: ${OpenCV_LIBS}")
message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")

include_directories(${OpenCV_INCLUDE_DIRS})
add_executable(videoStreaming videoStreaming.cpp)
add_executable(faceDetection faceDetection.cpp)
target_link_libraries(videoStreaming ${OpenCV_LIBS})
target_link_libraries(faceDetection ${OpenCV_LIBS})