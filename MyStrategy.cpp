#include <iostream>
#include "MyStrategy.h"
#include "State.h"
#include "VisClient.h"

using namespace model;
using namespace std;

VisClient *vis = nullptr;

void initialize() {
    vis = new VisClient(29292);
    vis->send("hello");
}

void MyStrategy::move(const Car& self, const World& world, const Game& game, Move& move) {
    static bool initialized = false;
    if (!initialized) {
        initialized = true;
        initialize();
    }

    move.setEnginePower(1.0);
    move.setThrowProjectile(true);
    move.setSpillOil(true);

    if (world.getTick() > game.getInitialFreezeDurationTicks()) {
        move.setUseNitro(true);
    }
}

MyStrategy::MyStrategy() { }
