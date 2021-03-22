cmake_minimum_required(VERSION 3.0)
project(cpp-heapMemory)

set(SRC_DIR .)
set(LIBS '')

include_directories(${SRC_DIR})

set (   SRC_FILES 
        Cube.cpp 
    )

# ./main
add_executable(main3 main.cpp ${SRC_FILES})
#target_link_libraries(main3 ${LIBS})

# ./heap1
add_executable(heap1 heap1.cpp ${SRC_FILES})
#target_link_libraries(heap1 ${LIBS})

# ./heap2
add_executable(heap2 heap2.cpp ${SRC_FILES})
#target_link_libraries(heap2 ${LIBS})
