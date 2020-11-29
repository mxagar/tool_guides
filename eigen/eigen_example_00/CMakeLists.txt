cmake_minimum_required(VERSION 3.0 FATAL_ERROR)

project(eigen_examples)

#find_package(Eigen3 REQUIRED)
#include_directories(${Eigen3_INCLUDE_DIRS})
#add_definitions(${Eigen3_DEFINITIONS})

if(WIN32)
    include_directories(/usr/local/include/eigen3)
elseif(LINUX)
    include_directories(/usr/local/include/eigen3)
elseif(APPLE)
    include_directories(/usr/local/include/eigen3)
endif()

add_executable (eigen_examples eigen_examples.cpp)

# This would be equivalent to
# g++ -I /path/to/eigen/ my_program.cpp -o my_program