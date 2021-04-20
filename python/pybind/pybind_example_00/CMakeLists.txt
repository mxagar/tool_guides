cmake_minimum_required(VERSION 3.5)

set(TARGET dummylib)

project(${TARGET})

find_package(Python 3.9 EXACT COMPONENTS Development)
#find_package(Python COMPONENTS Development)

add_subdirectory(pybind11)

file(GLOB SRC *.cpp)
file(GLOB HEADERS *.h)

pybind11_add_module(${TARGET} ${SRC} ${HEADERS})
