cmake_minimum_required(VERSION 3.0)
project(cpp-class)

set(SRC_DIR .)
set(LIBS '')

include_directories(${SRC_DIR})

set (   SRC_FILES 
        Cube.cpp 
    )

# ./main
add_executable(main main.cpp ${SRC_FILES})
target_link_libraries(main ${LIBS})

