cmake_minimum_required(VERSION 3.0)
project(cpp-class)

set(SRC_DIR .)
set(LIBS )

include_directories(${SRC_DIR})

set (   SRC_FILES 
        Cube.cpp 
    )

# ./main
add_executable(main main.cpp ${SRC_FILES})

# ./cout
add_executable(cout cout.cpp)
target_link_libraries(main ${LIBS})

# ./cout2
add_executable(cout2 cout2.cpp)
target_link_libraries(main ${LIBS})
