cmake_minimum_required(VERSION 3.0)
project(binary-tree-traversals)

set(SRC_DIR .)
set(LIBS '')

include_directories(${SRC_DIR})

set (   SRC_FILES 
#        ValueBinaryTree.hpp
    )

# Compilation -> add_executable / add_library
set (CMAKE_CXX_FLAGS "-std=c++14 -O0 -pedantic -Wall -Wfatal-errors -Wextra")
# Linking -> target_link_libraries
set (LDFLAGS "-std=c++14")

# ./binary-tree
add_executable(binary-tree-traversals main.cpp ${SRC_FILES})
#target_link_libraries(binary-tree-traversals ${LIBS})
