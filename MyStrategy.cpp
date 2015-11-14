#include "Const.h"
#include "MyStrategy.h"
#include "State.h"
#include "VisClient.h"

#include <iostream>
#include <unordered_map>

using namespace model;
using namespace std;

#define DEBUG_PHYSICS_PREDICTION
#ifndef ONLINE_JUDGE
#define VISUALIZE
#endif

VisClient *vis = nullptr;

void initialize(const Game& game) {
#ifdef VISUALIZE
    vis = new VisClient(29292);
#else
    vis = new VisClient(-1);
#endif
    vis->send("hello");

    Const::getInstance().game = game;
    cout.precision(8);
    cout << fixed;
    cerr.precision(8);
    cerr << fixed;
}

// ---- Debugging physics prediction ----

Go goDebugPhysicsPrediction(int tick) {
    if (280 <= tick && tick < 290) {
        return Go(1.0, 1.0);
    }
    return Go(1.0, 0.0);
};

bool isPhysicsPredictionOutputNeeded(int tick) {
    return 280 <= tick && tick <= 305;
}

void moveDebugPhysicsPrediction(const Car& self, const World& world, const Game& game, Move& move) {
    // This map is not entirely safe, one should not dereference originals of cars there
    static unordered_map<int, CarPosition> expectedPosByTick;

    auto experimentalMove = goDebugPhysicsPrediction(world.getTick());
    move.setEnginePower(experimentalMove.enginePower);
    move.setWheelTurn(experimentalMove.wheelTurn);
    move.setBrake(experimentalMove.brake);
    move.setThrowProjectile(experimentalMove.throwProjectile);
    move.setUseNitro(experimentalMove.useNitro);
    move.setSpillOil(experimentalMove.spillOil);

    const int lookahead = 1;

    auto currentState = State(&world);
    auto state = currentState;
    for (int i = 0; i < lookahead; i++) {
        auto go = goDebugPhysicsPrediction(world.getTick() + i);
        vector<Go> moves = { go, go, go, go };
        state = state.apply(moves);
    }

    auto expectedMyCar = state.getCarById(self.getId());
    expectedPosByTick.insert({ world.getTick() + lookahead, expectedMyCar });

    vis->drawPoly(expectedMyCar.getPoints());

    if (isPhysicsPredictionOutputNeeded(world.getTick())) {
        const double eps = 1e-9;
        auto currentInfo = expectedPosByTick.find(world.getTick());
        if (currentInfo == expectedPosByTick.end()) return;

        auto actual = currentState.getCarById(self.getId());
        auto predicted = currentInfo->second;
        cout << "tick " << world.getTick() << endl;
        cout << "  my position " << actual.toString() << endl;
        cout << "  predicted " << predicted.toString() << endl;
        cout << "  diff";
        auto diffLocation = actual.location.distanceTo(predicted.location);
        if (abs(diffLocation) > eps) cout << " location " << diffLocation;
        auto diffVelocity = actual.velocity - predicted.velocity;
        if (diffVelocity.length() > eps) cout << " velocity " << diffVelocity.toString();
        auto diffAngle = actual.angle - predicted.angle;
        if (abs(diffAngle) > eps) cout << " angle " << diffAngle;
        auto diffAngular = actual.angularSpeed - predicted.angularSpeed;
        if (abs(diffAngular) > eps) cout << " angular " << diffAngular;
        auto diffEnginePower = actual.enginePower - predicted.enginePower;
        if (abs(diffEnginePower) > eps) cout << " engine " << diffEnginePower;
        auto diffWheelTurn = actual.wheelTurn - predicted.wheelTurn;
        if (abs(diffWheelTurn) > eps) cout << " wheel " << diffWheelTurn;
        auto diffNitroCharges = actual.nitroCharges - predicted.nitroCharges;
        if (abs(diffNitroCharges) > eps) cout << " nitros " << diffNitroCharges;
        auto diffNitroCooldown = actual.nitroCooldown - predicted.nitroCooldown;
        if (abs(diffNitroCooldown) > eps) cout << " nitro cooldown " << diffNitroCooldown;
        cout << endl;
    }
}

// ----

void MyStrategy::move(const Car& self, const World& world, const Game& game, Move& move) {
    static bool initialized = false;
    if (!initialized) {
        initialized = true;
        initialize(game);
    }

#ifdef DEBUG_PHYSICS_PREDICTION
    moveDebugPhysicsPrediction(self, world, game, move);
    return;
#endif

    move.setEnginePower(1.0);
}

MyStrategy::MyStrategy() { }
