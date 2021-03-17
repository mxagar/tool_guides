cmake_minimum_required(VERSION 3.0 FATAL_ERROR)

project(eigen_examples)

#find_package(Eigen3 REQUIRED)
#include_directories(${Eigen3_INCLUDE_DIRS})
#add_definitions(${Eigen3_DEFINITIONS})

if(WIN32)
    include_directories(C:/src/Eigen3/eigen)
    # C1128 error issue in Windows
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /bigobj")
elseif(APPLE)
    include_directories(/usr/local/include/eigen3)
elseif(UNIX)
    include_directories(/usr/include/eigen3)
endif(WIN32)

add_executable (eigen_examples eigen_examples.cpp)

# This would be equivalent to
# g++ -I /path/to/eigen/ eigen_examples.cpp -o eigen_examples