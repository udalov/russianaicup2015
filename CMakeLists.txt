cmake_minimum_required(VERSION 2.6)
project(russianaicup)

SET(CMAKE_CXX_FLAGS "-D__APPLE__ -std=c++11 -O2 -Wall -Wno-unknown-pragmas -fno-optimize-sibling-calls -fno-strict-aliasing")

add_library(russianaicup
        csimplesocket/HTTPActiveSocket.cpp
        csimplesocket/ActiveSocket.cpp
        csimplesocket/PassiveSocket.cpp
        csimplesocket/SimpleSocket.cpp
        model/Bonus.cpp
        model/Car.cpp
        model/CircularUnit.cpp
        model/Game.cpp
        model/Move.cpp
        model/OilSlick.cpp
        model/Player.cpp
        model/PlayerContext.cpp
        model/Projectile.cpp
        model/RectangularUnit.cpp
        model/Unit.cpp
        model/World.cpp
        MyStrategy.cpp
        RemoteProcessClient.cpp
        Runner.cpp
        Strategy.cpp
        math2d.cpp
        State.cpp
        Go.cpp
        VisClient.cpp)

add_subdirectory(test)

add_executable(Runner runner-main.cpp)
add_executable(my-main main.cpp)

target_link_libraries(Runner russianaicup)
target_link_libraries(my-main russianaicup)
