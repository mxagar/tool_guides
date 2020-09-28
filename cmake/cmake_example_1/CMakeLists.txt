#Top-Level CMakeLists.txt in src/

# Project name all all variables are inherited from children folder
PROJECT(HELLO)

# Set Debug as build type to compile with debugging symbols & info
SET(CMAKE_BUILD_TYPE Debug)

# Children folder which will be visited
ADD_SUBDIRECTORY(HelloLib)
ADD_SUBDIRECTORY(TestBin)
