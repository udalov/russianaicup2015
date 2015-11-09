SET(CMAKE_CXX_FLAGS "-D__APPLE__ -std=c++11 -O2 -Wall -Wno-unknown-pragmas")
# SET(GTEST_COLOR "auto")

include_directories(${gtest_SOURCE_DIR}/include ${gtest_SOURCE_DIR})
include_directories(..)

add_subdirectory(gtest-1.7.0)

add_executable(tests Math2DTest.cpp)

target_link_libraries(tests gtest gtest_main)
target_link_libraries(tests russianaicup)
