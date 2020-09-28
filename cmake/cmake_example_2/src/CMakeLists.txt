#
# Sample C++ project compiled using CMake
# DLR, Robotics und Mechatronics
# Mikel Sagardia, 2013 (c)
#

CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

PROJECT(MIKELTEST)


SET(MIKELTEST_VERSION_MAJOR 1)
SET(MIKELTEST_VERSION_MINOR 2)
SET(MIKELTEST_VERSION_PATCH 3)

SET(DLR_BUILD ON CACHE BOOL "This variable is ON when project built at DLR")

IF(WIN32)
	#Put here all necessary variables, similar to UNIX
ELSEIF(APPLE)
	#Put here all necessary variables
ELSEIF(UNIX)
	IF(DLR_BUILD)
		SET(LIB1_INCDIR ${CMAKE_SOURCE_DIR}/lib_1 CACHE PATH "Path lib1 includes")
		SET(LIB1_LIBDIR ${CMAKE_SOURCE_DIR}/lib_1 CACHE PATH "Path lib1 libraries")
		SET(LIB2_INCDIR ${CMAKE_SOURCE_DIR}/lib_2 CACHE PATH "Path to bullet libs")
		SET(LIB2_LIBDIR ${CMAKE_SOURCE_DIR}/lib_2 CACHE PATH "Path to bullet libs")
	ENDIF(DLR_BUILD)
ENDIF(WIN32)

SET(PROJECT_INCDIR ${CMAKE_SOURCE_DIR}/../include CACHE PATH "Path of project includes")
SET(PROJECT_LIBDIR ${CMAKE_SOURCE_DIR}/../lib CACHE PATH "Path of project libraries")
SET(PROJECT_BINDIR ${CMAKE_SOURCE_DIR}/../bin CACHE PATH "Path project binaries")
SET(CMAKE_INSTALL_PREFIX ${PROJECT_BINDIR} CACHE PATH "Path to install the project executables" FORCE)

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
