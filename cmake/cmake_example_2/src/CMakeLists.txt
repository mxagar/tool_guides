#
# Sample C++ project compiled using CMake
# Mikel Sagardia, 2013 (c)
#

CMAKE_MINIMUM_REQUIRED(VERSION 3.0)

# Project name can be accessed with ${CMAKE_PROJECT_NAME}
PROJECT("CMake_Example")

# Set output paths; these are created and populated eventhough we don't do 'make install'
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/bin)

# Define library and include paths
SET(LIB1_INCDIR ${CMAKE_SOURCE_DIR}/lib_1 CACHE PATH "Path lib1 includes")
SET(LIB1_LIBDIR ${CMAKE_SOURCE_DIR}/lib_1 CACHE PATH "Path lib1 libraries")
SET(LIB2_INCDIR ${CMAKE_SOURCE_DIR}/lib_2 CACHE PATH "Path lib2 includes")
SET(LIB2_LIBDIR ${CMAKE_SOURCE_DIR}/lib_2 CACHE PATH "Path lib2 libraries")

# Platform specific defines
IF(WIN32)
	#Put here all necessary variables
ELSEIF(APPLE)
	#Put here all necessary variables
ELSEIF(UNIX)
	#Put here all necessary variables
ENDIF(WIN32)

# Independently of the output paths, we can define some other directories for the installation
SET(PROJECT_INCDIR ${CMAKE_SOURCE_DIR}/../include CACHE PATH "Path of project includes")
SET(PROJECT_LIBDIR ${CMAKE_SOURCE_DIR}/../lib CACHE PATH "Path of project libraries")
SET(PROJECT_BINDIR ${CMAKE_SOURCE_DIR}/../bin CACHE PATH "Path project binaries")
SET(CMAKE_INSTALL_PREFIX ${PROJECT_BINDIR} CACHE PATH "Path to install the project executables" FORCE)

# STATIC / SHARED libraries? ON: STATIC, OFF: SHARED
# Watch out: if SHARED + Windows, library headers need need a macro with __declspec() which preceeds any function & class definition
option(PROJECT_BUILD_STATIC "Build static library" ON)
#option(PROJECT_BUILD_SHARED "Build shared library" OFF)

SET(BUILD_LIB1 ON CACHE BOOL "This variable is ON when the lib1 should be built")
IF(BUILD_LIB1)
	SET(LIB1_NAME "lib1")
	ADD_SUBDIRECTORY(${CMAKE_SOURCE_DIR}/lib_1)
ENDIF(BUILD_LIB1)

SET(BUILD_LIB2 ON CACHE BOOL "This variable is ON when the lib2 should be built")
IF(BUILD_LIB2)
	SET(LIB2_NAME "lib2")
	ADD_SUBDIRECTORY(${CMAKE_SOURCE_DIR}/lib_2)
ENDIF(BUILD_LIB2)

SET(BUILD_APP1 ON CACHE BOOL "This variable is ON when the app1 should be built")
IF(BUILD_APP1)
	SET(APP1_NAME "app1")
	ADD_SUBDIRECTORY(${CMAKE_SOURCE_DIR}/app_1)
ENDIF(BUILD_APP1)

SET(BUILD_APP2 ON CACHE BOOL "This variable is ON when the app2 should be built")
IF(BUILD_APP2)
	SET(APP2_NAME "app2")
	ADD_SUBDIRECTORY(${CMAKE_SOURCE_DIR}/app_2)
ENDIF(BUILD_APP2)

SET(BUILD_DOXYGEN_DOCU ON CACHE BOOL "This variable is ON when documentation must be built")
IF(BUILD_DOXYGEN_DOCU)
	ADD_SUBDIRECTORY(${CMAKE_SOURCE_DIR}/doc)
ENDIF(BUILD_DOXYGEN_DOCU)
