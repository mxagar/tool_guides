cmake_minimum_required(VERSION 3.0)
project(CoreFunctionalities)

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
# Executable 1: default program from the samples to create Charuco markers
add_executable(create_marker create_marker.cpp)
target_link_libraries(create_marker ${OpenCV_LIBS})
# Executable 2: default program from the samples to create Charuco markers
add_executable(detect_markers detect_markers.cpp)
target_link_libraries(detect_markers ${OpenCV_LIBS})
# Executable 3: program I made to detect Charuco markers from an image
add_executable(simple_detect_markers_image simple_detect_markers_image.cpp)
target_link_libraries(simple_detect_markers_image ${OpenCV_LIBS})
# Executable 4: program I made to detect Charuco markers from a video stream
add_executable(simple_detect_markers_videostream simple_detect_markers_videostream.cpp)
target_link_libraries(simple_detect_markers_videostream ${OpenCV_LIBS})
# Executable 5: default program from the samples to create Charuco board
add_executable(create_board_charuco create_board_charuco.cpp)
target_link_libraries(create_board_charuco ${OpenCV_LIBS})
# Executable 6: default program from the samples to detect Charuco board
add_executable(detect_board_charuco detect_board_charuco.cpp)
target_link_libraries(detect_board_charuco ${OpenCV_LIBS})
# Executable 7: default program from the samples to calibrate the camera with a Charuco board (intrinsic cam params obtained)
add_executable(calibrate_camera_charuco calibrate_camera_charuco.cpp)
target_link_libraries(calibrate_camera_charuco ${OpenCV_LIBS})
