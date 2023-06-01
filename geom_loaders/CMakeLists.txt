cmake_minimum_required(VERSION 3.0)
project(GeometryLoader)

set(INCLUDE_DIRECTORIES 
    .
    include
    )
include_directories(${INCLUDE_DIRECTORIES})

# Compilation -> add_executable / add_library
set (CMAKE_CXX_FLAGS "-std=c++14 -O0 -pedantic -Wall -Wfatal-errors -Wextra")
# Linking -> target_link_libraries
set (LDFLAGS "-std=c++14")

add_executable(geom_loader geom_loaders_examples.cpp)