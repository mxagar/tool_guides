cmake_minimum_required(VERSION 3.0)
project(cpp-std)

set(SRC_DIR .)
set(LIBS '')

include_directories(${SRC_DIR})

set (   SRC_FILES 
        Cube.cpp 
    )

# Compilation -> add_executable / add_library
set (CMAKE_CXX_FLAGS "-std=c++14 -O0 -pedantic -Wall -Wfatal-errors -Wextra")
# Linking -> target_link_libraries
set (LDFLAGS "-std=c++14")

# ./cout
add_executable(cout cout.cpp)
#target_link_libraries(cout ${LIBS})

# ./cout2
add_executable(cout2 cout2.cpp)
#target_link_libraries(cout2 ${LIBS})
