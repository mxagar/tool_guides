cmake_minimum_required(VERSION 3.0)
project(week-1-challenge)

set(SRC_DIR .)
set(LIBS '')

include_directories(${SRC_DIR})

set (   SRC_FILES 
#        Cube.cpp
    )

# Compilation -> add_executable / add_library
set (CMAKE_CXX_FLAGS "-std=c++14 -O0 -pedantic -Wall -Wfatal-errors -Wextra")
# Linking -> target_link_libraries
set (LDFLAGS "-std=c++14")

# ./stack
add_executable(week1challenge main.cpp ${SRC_FILES})
#target_link_libraries(week1challenge ${LIBS})
