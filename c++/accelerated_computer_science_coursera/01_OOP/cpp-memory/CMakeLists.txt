cmake_minimum_required(VERSION 3.0)
project(cpp-memory)

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

# ./main
add_executable(main2 main.cpp ${SRC_FILES})
#target_link_libraries(main2 ${LIBS})

# ./foo
add_executable(foo foo.cpp ${SRC_FILES})
#target_link_libraries(foo ${LIBS})

# ./addressOf
add_executable(addressOf addressOf.cpp ${SRC_FILES})
#target_link_libraries(addressOf ${LIBS})

# ./puzzle
add_executable(puzzle puzzle.cpp ${SRC_FILES})
#target_link_libraries(puzzle ${LIBS})
