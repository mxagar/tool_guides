cmake_minimum_required(VERSION 3.0)
project(week-4-challenge)

set(SRC_DIR .)
set(LIBS '')

include_directories(${SRC_DIR})

set (   SRC_FILES 
    )

# Compilation -> add_executable / add_library
set (CMAKE_CXX_FLAGS "-std=c++14 -O0 -pedantic -Wall -Wfatal-errors -Wextra")
# Linking -> target_link_libraries
set (LDFLAGS "-std=c++14")

# ./week_2_challenge
add_executable(week_4_challenge main.cpp)
#target_link_libraries(cout ${LIBS})