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

Go moveForTick(int tick) {
    if (280 <= tick && tick < 290) {
        return Go(1.0, 1.0);
    }
    return Go(1.0, 0.0);
};

// TODO: not safe
unordered_map<int, CarPosition> expectedPosByTick;

void MyStrategy::move(const Car& self, const World& world, const Game& game, Move& move) {
    static bool initialized = false;
    if (!initialized) {
        initialized = true;
        initialize(game);
    }

    auto experimentalMove = moveForTick(world.getTick());
    move.setEnginePower(experimentalMove.enginePower);
    move.setWheelTurn(experimentalMove.wheelTurn);
    move.setThrowProjectile(true);
    move.setSpillOil(true);

    const int lookahead = 1;

    auto currentState = State(&world);
    auto state = currentState;
    for (int i = 0; i < lookahead; i++) {
        auto go = moveForTick(world.getTick() + i);
        vector<Go> moves = { go, go, go, go };
        state = state.apply(moves);
    }
    
    auto expectedMyCar = getCarById(state, self.getId());
    expectedPosByTick.insert({ world.getTick() + lookahead, expectedMyCar });

    drawPoly(expectedMyCar.getPoints());

    if (280 <= world.getTick() && world.getTick() <= 300) {
        auto currentInfo = expectedPosByTick.find(world.getTick());
        if (currentInfo != expectedPosByTick.end()) {
            auto actual = getCarById(currentState, self.getId());
            auto predicted = currentInfo->second;
            cout << "tick " << world.getTick() << endl;
            cout << "  my position " << actual.toString() << endl;
            cout << "  predicted " << predicted.toString() << endl;
            cout.precision(8);
            cout << fixed << "  diff location " << actual.location.distanceTo(predicted.location) <<
                    " velocity " << (actual.velocity - predicted.velocity).toString() <<
                    " angle " << abs(actual.angle - predicted.angle) <<
                    " angular " << abs(actual.angularSpeed - predicted.angularSpeed) <<
                    " engine " << abs(actual.enginePower - predicted.enginePower) <<
                    " wheel " << abs(actual.wheelTurn - predicted.wheelTurn) << endl;
        }
    }
}

MyStrategy::MyStrategy() { }
