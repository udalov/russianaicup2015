cmake_minimum_required(VERSION 2.6)
project(russianaicup)

if(CMAKE_SYSTEM MATCHES "Linux.*")
    set(CMAKE_CXX_FLAGS "-D_LINUX -std=c++11 -O2 -g -pthread -lpthread -Wall -Wno-unknown-pragmas -fno-optimize-sibling-calls -fno-strict-aliasing")
else()
    set(CMAKE_CXX_FLAGS "-D__APLE__ -std=c++11 -O2 -g -Wall -Wno-unknown-pragmas -fno-optimize-sibling-calls -fno-strict-aliasing")
endif()

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
        math3d.cpp
        State.cpp
        Go.cpp
        VisClient.cpp
        Const.cpp
        Map.cpp
        Collider.cpp
        Path.cpp
        Scorer.cpp
        Tracks.cpp
        Debug.cpp
        )

add_subdirectory(test)

add_executable(Runner runner-main.cpp)
add_executable(LocalRunner main.cpp)

target_link_libraries(Runner russianaicup)
target_link_libraries(LocalRunner russianaicup)
