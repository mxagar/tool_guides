cmake_minimum_required(VERSION 3.0)
project(cpp-memory2-ex2)

set(SRC_DIR .)
set(LIBS '')

include_directories(${SRC_DIR} ${SRC_DIR}/..)

set (   SRC_FILES 
        ../Cube.cpp
    )

# Compilation -> add_executable / add_library
set (CMAKE_CXX_FLAGS "-std=c++14 -O0 -pedantic -Wall -Wfatal-errors -Wextra")
# Linking -> target_link_libraries
set (LDFLAGS "-std=c++14")

# ./byValue2
add_executable(byValue2 byValue.cpp ${SRC_FILES})
#target_link_libraries(byValue2 ${LIBS})

# ./byPointer2
add_executable(byPointer2 byPointer.cpp ${SRC_FILES})
#target_link_libraries(byPointer2 ${LIBS})

# ./byRef2
add_executable(byRef2 byRef.cpp ${SRC_FILES})
#target_link_libraries(byRef2 ${LIBS})
