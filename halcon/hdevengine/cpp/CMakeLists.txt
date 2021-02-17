cmake_minimum_required(VERSION 3.0 FATAL_ERROR)

project(hdevengine_examples)

set (SRC_DIR . )

if(UNIX)
    set (HALCON_INCLUDE_DIR . )
    set (HALCON_LIB_DIR . )
    set (HALCON_LIBS . )
elseif(WIN32)
    set (HALCON_INCLUDE_DIR . )
    set (HALCON_LIB_DIR . )
    set (HALCON_LIBS . )
elseif(APPLE)
    set (HALCON_INCLUDE_DIR /Library/Frameworks/HALCONCpp.framework/Headers )
    set (HALCON_LIB_DIR /Library/Frameworks/HALCONCpp.framework )
    set (HALCON_LIBS HALCONCpp )
endif(UNIX)

include_directories(${HALCON_INCLUDE_DIR})
link_directories(${HALCON_LIB_DIR})

# Executable 1
add_executable (exec_program exec_program.cpp)
target_link_libraries (exec_program ${HALCON_LIBS})