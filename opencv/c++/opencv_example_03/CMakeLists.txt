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

# Executable 1: 
add_executable(camera_calibration 01_camera_calibration.cpp)
target_link_libraries(camera_calibration ${OpenCV_LIBS})

# Executable 2: 
add_executable(calibration 02_calibration.cpp)
target_link_libraries(calibration ${OpenCV_LIBS})

# Executable 3: 
add_executable(imagelist_creator imagelist_creator.cpp)
target_link_libraries(imagelist_creator ${OpenCV_LIBS})

# Executable 4: 
add_executable(imagelist_reader imagelist_reader.cpp)
target_link_libraries(imagelist_reader ${OpenCV_LIBS})
