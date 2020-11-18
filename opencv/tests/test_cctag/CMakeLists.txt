cmake_minimum_required(VERSION 3.0)

project(cctagtest)

# Work-around for Windows / vcpkg packages
if (WIN32)
    set (CMAKE_TOOLCHAIN_FILE "C:/src/vcpkg/scripts/buildsystems/vcpkg.cmake")
    set (CCTag_DIR "C:/Program\ Files\ \(x86\)/CCTag/lib/cmake/CCTag")
    set (Boost_DIR "C:/src/vcpkg/installed/x64-windows/bin")
    set (Boost_INCLUDE_DIR "C:/src/vcpkg/installed/x64-windows/include")
endif(WIN32)

# Find the package from the CCTagConfig.cmake
# in <prefix>/lib/cmake/CCTag/. Under the namespace CCTag::
# it exposes the target CCTag that allows you to compile
# and link with the library
find_package(CCTag CONFIG REQUIRED)
# suppose you want to try it out in a executable
add_executable(cctagtest test_cctag.cpp)
# add link to the library
target_link_libraries(cctagtest PUBLIC CCTag::CCTag)
