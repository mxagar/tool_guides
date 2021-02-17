cmake_minimum_required(VERSION 3.0 FATAL_ERROR)

project(hdevengine_examples)

set (SRC_DIR . )

if(UNIX)
    set (HALCON_INCLUDE_DIRS . )
    set (HALCON_LIB_DIR . )
    set (HALCON_LIBS . )
elseif(WIN32)
    # Set HALCON variables manually
    set (HALCONROOT "C:/Program\ Files/MVTec/HALCON-20.05-Progress" )
    set (HALCONARCH "x64-win64" )
    set (HALCONEXAMPLES "C:/Users/Public/Documents/MVTec/HALCON-20.05-Progress/examples" )
    set (HALCONIMAGES ${HALCONEXAMPLES}"/images" )
    set (HALCON_INCLUDE_DIRS 
        ${SRC_DIR}
        ${HALCONROOT}/include
        ${HALCONROOT}/include/halconcpp
        ${HALCONROOT}/include/hdevengine 
    )
    set (HALCON_LIB_DIR ${HALCONROOT}/lib/${HALCONARCH} )
    set (HALCON_LIBS hdevenginecpp.lib halconcpp.lib halcon.lib )
elseif(APPLE)
    # I have not managed to make it work
    set (HALCON_INCLUDE_DIRS /Library/Frameworks/HALCONCpp.framework/Headers )
    set (HALCON_LIB_DIR /Library/Frameworks/HALCONCpp.framework )
    set (HALCON_LIBS HALCONCpp )
endif(UNIX)

include_directories(${HALCON_INCLUDE_DIRS})
link_directories(${HALCON_LIB_DIR})

# Executable 1
message("-> Include directories: " ${HALCON_INCLUDE_DIRS} )
add_executable (exec_program my_error_output.cpp my_thread_impl.cpp my_hdevoperatorimpl.cpp exec_program_simple.cpp)
target_link_libraries (exec_program ${HALCON_LIBS})