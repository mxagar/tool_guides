cmake_minimum_required(VERSION 3.0)
project(cpp-tower-solution2)

set(SRC_DIR .)
set(LIBS '')

include_directories( ${SRC_DIR} ${SRC_DIR}/uiuc )

set (   SRC_FILES 
        uiuc/Cube.cpp
        uiuc/HSLAPixel.cpp
        Game.cpp
        Stack.cpp
    )

# Compilation -> add_executable / add_library
set (CMAKE_CXX_FLAGS "-std=c++14 -O0 -pedantic -Wall -Wfatal-errors -Wextra")
# Linking -> target_link_libraries
set (LDFLAGS "-std=c++14")

# ./tower
add_executable(tower2 main.cpp ${SRC_FILES})
#target_link_libraries(tower2 ${LIBS})
