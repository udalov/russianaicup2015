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
    
    auto state = State(&world);
    auto go = Go(1.0, 0.0);
    auto moves = vector<Go> { go, go, go, go };
    for (int i = 0; i < 10; i++) {
        state = state.apply(moves);
    }

    auto myCar = find_if(state.cars.begin(), state.cars.end(), [&self](const CarPosition& car) {
        return car.original->getId() == self.getId();
    });
    if (myCar == state.cars.end()) myCar = state.cars.begin();
    auto points = myCar->getPoints();
    for (auto i = 0; i < points.size(); i++) {
        auto p = points[i];
        auto q = points[i + 1 == points.size() ? 0 : i + 1];
        vis->drawLine(p.x, p.y, q.x, q.y);
    }

    move.setEnginePower(1.0);
    move.setThrowProjectile(true);
    move.setSpillOil(true);

    if (world.getTick() > game.getInitialFreezeDurationTicks()) {
        move.setUseNitro(true);
    }
}

MyStrategy::MyStrategy() { }
