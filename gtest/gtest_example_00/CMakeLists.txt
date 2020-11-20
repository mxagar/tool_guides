cmake_minimum_required(VERSION 3.0)
project(basic_test)

################################
# GTest
################################
if (WIN32)
    set(GTest_INCLUDE_DIRS "C:/src/GTest/googletest-1.10.0/googletest/include")
    set(GTest_LIBRARY_DIRS "C:/src/GTest/googletest-1.10.0/googletest/build/lib/Release")
    #set(GTest_LIBRARY_DIRS "C:/src/GTest/googletest-1.10.0/googletest/build/bin/Release")
else()
    find_package(GTest CONFIG REQUIRED)
endif()

################################
# Tests
################################
# Add test cpp file
add_executable(test_gtest test_gtest.cpp)
# Link test executable against gtest & gtest_main
if (WIN32)
    include_directories(${GTest_INCLUDE_DIRS})
    link_directories(${GTest_LIBRARY_DIRS})
    target_link_libraries(test_gtest gtest gtest_main)
else()
    target_link_libraries(test_gtest GTest::gtest GTest::gtest_main)    
endif()