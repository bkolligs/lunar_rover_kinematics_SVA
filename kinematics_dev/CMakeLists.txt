cmake_minimum_required(VERSION 2.6)
SET(GCC_COVERAGE_COMPILE_FLAGS "-O1 -O2")
SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} ${GCC_COVERAGE_COMPILE_FLAGS}")
SET(CMAKE_EXE_LINKER_FLAGS  "${CMAKE_EXE_LINKER_FLAGS} ${GCC_COVERAGE_LINK_FLAGS}")
 
# Locate GTest
find_package(GTest REQUIRED)
include_directories(
    include
    test/include_tests
    ${GTEST_INCLUDE_DIRS})

# create library with the test file for the header
add_library(kinematics
    include/kinematics.h
    src/kinematics.cpp
    include/classTest.h
    src/classTest.cpp)


 
# compile the executable matrices.cpp
add_executable(rover_kinematics src/main.cpp)
target_link_libraries(rover_kinematics ${GTEST_LIBRARIES} pthread kinematics)

# testing executables
# create library with the test file for the header
 
# Link runTests with what we want to test and the GTest and pthread library
add_executable(runTests test/kinematics_tests.cpp)
target_link_libraries(runTests ${GTEST_LIBRARIES} pthread kinematics)
