cmake_minimum_required(VERSION 3.0)
project(cpp-heapPuzzles)

set(SRC_DIR .)
set(LIBS '')

include_directories(${SRC_DIR})

set (   SRC_FILES 
    )

# Compilation -> add_executable / add_library
set (CMAKE_CXX_FLAGS "-std=c++14 -O0 -pedantic -Wall -Wfatal-errors -Wextra")
# Linking -> target_link_libraries
set (LDFLAGS "-std=c++14")

# ./puzzle1
add_executable(puzzle1 puzzle1.cpp ${SRC_FILES})
#target_link_libraries(puzzle1 ${LIBS})

# ./puzzle2
add_executable(puzzle2 puzzle2.cpp ${SRC_FILES})
#target_link_libraries(puzzle2 ${LIBS})

# ./puzzle3
add_executable(puzzle3 puzzle3.cpp ${SRC_FILES})
#target_link_libraries(puzzle3 ${LIBS})

# ./puzzle4
add_executable(puzzle4 puzzle4.cpp ${SRC_FILES})
#target_link_libraries(puzzle4 ${LIBS})
