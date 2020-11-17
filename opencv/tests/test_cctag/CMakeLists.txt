cmake_minimum_required(VERSION 3.0)
	
# Find the package from the CCTagConfig.cmake
# in <prefix>/lib/cmake/CCTag/. Under the namespace CCTag::
# it exposes the target CCTag that allows you to compile
# and link with the library
find_package(CCTag CONFIG REQUIRED)
# suppose you want to try it out in a executable
add_executable(cctagtest test_cctag.cpp)
# add link to the library
target_link_libraries(cctagtest PUBLIC CCTag::CCTag)
