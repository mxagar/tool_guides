cmake_minimum_required(VERSION 3.0)
project(cpp-threads)

set(SRC_DIR .)
set(LIBS '')

include_directories( ${SRC_DIR} )

set (   SRC_FILES 
        #ObjectCode.cpp
    )

# Compilation -> add_executable / add_library
#set (CMAKE_CXX_FLAGS "-std=c++14 -O0 -pedantic -Wall -Wfatal-errors -Wextra")
set (CMAKE_CXX_FLAGS "-std=c++14 -O0 -pedantic -Wall")
# Linking -> target_link_libraries
set (LDFLAGS "-std=c++14")

# ./simple_mutex
add_executable(simple_mutex simple_mutex.cpp ${SRC_FILES})
#target_link_libraries(simple_mutex ${LIBS})

# ./simple_mutex
add_executable(producer_consumer producer_consumer.cpp ${SRC_FILES})
#target_link_libraries(producer_consumer ${LIBS})
