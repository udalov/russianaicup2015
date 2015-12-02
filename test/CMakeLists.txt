if(CMAKE_SYSTEM MATCHES "Linux.*")
    set(CMAKE_CXX_FLAGS "-D_LINUX -std=c++11 -O2 -Wall -Wno-unknown-pragmas")
else()
    set(CMAKE_CXX_FLAGS "-D__APPLE__ -std=c++11 -O2 -Wall -Wno-unknown-pragmas")
endif()

# set(GTEST_COLOR "auto")

include_directories(gtest-1.7.0/include gtest-1.7.0)
include_directories(..)

add_subdirectory(gtest-1.7.0)

add_executable(tests Math2DTest.cpp)

target_link_libraries(tests gtest gtest_main)
target_link_libraries(tests russianaicup)
