#include <iostream>
#include <unordered_map>
#include "Const.h"
#include "MyStrategy.h"
#include "State.h"
#include "VisClient.h"

using namespace model;
using namespace std;

VisClient *vis = nullptr;

void initialize(const Game& game) {
    vis = new VisClient(29292);
    vis->send("hello");

    Const::getInstance().game = game;
}

CarPosition getCarById(const State& state, long long id) {
    auto car = find_if(state.cars.begin(), state.cars.end(), [&id](const CarPosition& it) {
        return it.original->getId() == id;
    });
    return car == state.cars.end() ? state.cars.front() : *car;
}

void drawPoly(const vector<Point>& points) {
    for (auto i = 0; i < points.size(); i++) {
        auto p = points[i];
        auto q = points[i + 1 == points.size() ? 0 : i + 1];
        vis->drawLine(p.x, p.y, q.x, q.y);
    }
}

// TODO: not safe
unordered_map<int, CarPosition> expectedPosByTick;

void MyStrategy::move(const Car& self, const World& world, const Game& game, Move& move) {
    static bool initialized = false;
    if (!initialized) {
        initialized = true;
        initialize(game);
    }

    if (280 < world.getTick() && world.getTick() < 290) {
        move.setWheelTurn(1.0);
    }

    move.setEnginePower(1.0);
    move.setThrowProjectile(true);
    move.setSpillOil(true);

    const int lookahead = 10;

    auto currentState = State(&world);
    auto go = Go(move.getEnginePower(), move.getWheelTurn());
    auto moves = vector<Go> { go, go, go, go };
    auto state = currentState;
    for (int i = 0; i < lookahead; i++) {
        state = state.apply(moves);
    }
    
    auto expectedMyCar = getCarById(state, self.getId());
    expectedPosByTick.insert({ world.getTick() + lookahead, expectedMyCar });

    drawPoly(expectedMyCar.getPoints());

/*
    if (280 < world.getTick() && world.getTick() < 330) {
        auto currentInfo = expectedPosByTick.find(world.getTick());
        if (currentInfo != expectedPosByTick.end()) {
            cout << "tick " << world.getTick() << endl;
            cout << "  my position " << getCarById(currentState, self.getId()).toString() << endl;
            cout << "  predicted " << currentInfo->second.toString() << endl;
        }
    }
*/
}

MyStrategy::MyStrategy() { }
