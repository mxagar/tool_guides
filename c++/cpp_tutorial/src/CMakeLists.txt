cmake_minimum_required(VERSION 3.0)
project(cpp-tutorial-advanced)

set(SRC_DIR .)
set(LIBS '')

include_directories( ${SRC_DIR} )

set (   SRC_FILES_EXE
        objectCode.cpp
    )

set (   SRC_FILES_SHARED_LIB
        dynamicCode.cpp
)

set (   SRC_FILES_STATIC_LIB
        staticCode.cpp
)

# Compilation -> add_executable / add_library
set (CMAKE_CXX_FLAGS "-std=c++14 -O0 -pedantic -Wall -Wfatal-errors -Wextra")
# Linking -> target_link_libraries
set (LDFLAGS "-std=c++14")

add_library(Dynamic SHARED ${SRC_FILES_SHARED_LIB})
add_library(Static ${SRC_FILES_STATIC_LIB})

# ./mainApp
add_executable(mainApp mainApp.cpp ${SRC_FILES_EXE})
target_link_libraries(mainApp Static Dynamic)
