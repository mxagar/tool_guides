cmake_minimum_required(VERSION 3.0)
project(cpp-memory2-ex1)

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

# ./byValue
add_executable(byValue byValue.cpp ${SRC_FILES})
#target_link_libraries(byValue ${LIBS})

# ./byPointer
add_executable(byPointer byPointer.cpp ${SRC_FILES})
#target_link_libraries(byPointer ${LIBS})

# ./byRef
add_executable(byRef byRef.cpp ${SRC_FILES})
#target_link_libraries(byRef ${LIBS})