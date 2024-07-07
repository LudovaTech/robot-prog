cmake_minimum_required(VERSION 3.24)
project("robot-prog")

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# include Google Test
include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/refs/tags/v1.14.0.zip
)

set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

# faking Arduino.h
include_directories(./test/faking/)
add_library(Arduino STATIC ./test/faking/Arduino.cpp)
target_include_directories(Arduino PUBLIC ./test/faking)

# include own test files
# file(GLOB TEST_SOURCES "test/*_unit.cpp")
set(TEST_SOURCES "test/fake_arduino_unit.cpp" "test/lidar_unit.cpp" "test/movements_unit.cpp" "test/utilities_unit.cpp" "test/cam_reader_unit.cpp")
# file(GLOB SRC_SOURCES "src/*.cpp")
# add_executable(unitTest ${TEST_SOURCES} ${SRC_SOURCES})
add_executable(unitTest ${TEST_SOURCES} ./src/utilities.cpp ./src/cam_reader.cpp ./src/strategy.cpp ./src/movements.cpp ./src/lidar_reader.cpp ./src/logger.cpp)

target_link_libraries(unitTest gtest_main Arduino)
add_test(NAME example_test COMMAND unitTest)
