cmake_minimum_required(VERSION 3.0)
project(cpp-std)

set(SRC_DIR .)
set(LIBS '')

include_directories(${SRC_DIR})

set (   SRC_FILES 
        Cube.cpp 
    )

# ./cout
add_executable(cout cout.cpp)
target_link_libraries(cout ${LIBS})

# ./cout2
add_executable(cout2 cout2.cpp)
target_link_libraries(cout2 ${LIBS})
