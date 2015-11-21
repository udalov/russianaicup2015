#include "math2d.h"
#include "MyStrategy.h"
#include "Const.h"
#include "Map.h"
#include "Path.h"
#include "State.h"
#include "VisClient.h"

#include <iostream>
#include <sstream>
#include <unordered_map>

using namespace model;
using namespace std;

// #define DEBUG_PHYSICS_PREDICTION
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

enum class DPPMode {
    COLLISION_WITH_SEGMENT_HIGH_SPEED,
    COLLISION_WITH_SEGMENT_SIDEWAYS,
    COLLISION_WITH_CORNER,
    BRAKE,
};

const DPPMode mode = DPPMode::BRAKE;

Go goDebugPhysicsPrediction(int tick) {
    if (mode == DPPMode::COLLISION_WITH_SEGMENT_HIGH_SPEED) {
        if (tick == 181) {
            return Go(1.0, 1.0, false, false, true);
        }
        if (280 <= tick && tick < 285) {
            return Go(1.0, 1.0);
        }
        return Go(1.0, 0.0);
    } else if (mode == DPPMode::COLLISION_WITH_SEGMENT_SIDEWAYS) {
        if (400 <= tick && tick < 450) {
            return Go(-1.0, 1.0);
        }
        return Go(1.0, 0.0);
    } else if (mode == DPPMode::COLLISION_WITH_CORNER) {
        if (375 <= tick && tick < 450) {
            return Go(-1.0, 1.0);
        }
        if (450 <= tick && tick < 480) {
            return Go(-1.0, 0.0);
        }
        return Go(1.0, 0.0);
    } else if (mode == DPPMode::BRAKE) {
        if (250 <= tick && tick <= 260) {
            return Go(1.0, 0.0, true);
        }
        return Go(1.0, 0.0);
    }
};

bool isPhysicsPredictionOutputNeeded(int tick) {
    if (mode == DPPMode::COLLISION_WITH_SEGMENT_HIGH_SPEED) {
        return 359 <= tick && tick <= 363;
    } else if (mode == DPPMode::COLLISION_WITH_SEGMENT_SIDEWAYS) {
        return 461 <= tick && tick <= 465;
    } else if (mode == DPPMode::COLLISION_WITH_CORNER) {
        return 609 <= tick && tick <= 613;
    } else if (mode == DPPMode::BRAKE) {
        return 248 <= tick && tick <= 262;
    } else {
        terminate();
    }
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

    auto currentState = State(&world);

    if (isPhysicsPredictionOutputNeeded(world.getTick())) {
        const double eps = 1e-9;
        auto currentInfo = expectedPosByTick.find(world.getTick());
        if (currentInfo == expectedPosByTick.end()) return;

        auto actual = currentState.myCar();
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
        auto diffHealth = actual.health - predicted.health;
        if (abs(diffHealth) > eps) cout << " health " << diffHealth;
        auto diffNitroCharges = actual.nitroCharges - predicted.nitroCharges;
        if (abs(diffNitroCharges) > eps) cout << " nitros " << diffNitroCharges;
        auto diffNitroCooldown = actual.nitroCooldown - predicted.nitroCooldown;
        if (abs(diffNitroCooldown) > eps) cout << " nitro cooldown " << diffNitroCooldown;
        cout << endl;
    }

    const int lookahead = 1;

    auto state = State(currentState);
    for (int i = 0; i < lookahead; i++) {
        auto go = goDebugPhysicsPrediction(world.getTick() + i);
        vector<Go> moves(world.getCars().size(), go);
        state.apply(moves);
    }

    expectedPosByTick.insert({ world.getTick() + lookahead, state.myCar() });

    vis->drawRect(state.myCar().rectangle());
}

// ----

void drawMap() {
    auto g = Map::getMap().graph;
    const int dx[] = {1, 0, -1, 0};
    const int dy[] = {0, 1, 0, -1};
    const double s = 200.0;
    for (unsigned long i = 0; i < g.size(); i++) {
        for (unsigned long j = 0; j < g[i].size(); j++) {
            double x = i * s + s/2;
            double y = j * s + s/2;
            for (int d = 0; d < 4; d++) {
                if (g[i][j] & (1 << d)) {
                    vis->drawLine(Point(x, y), Point(x + s/2*dx[d], y + s/2*dy[d]));
                }
            }
        }
    }
}

// ----

// ----

Tile findCurrentTile(const Car& self) {
    auto& map = Map::getMap();
    auto& game = Const::getGame();
    const double tileSize = game.getTrackTileSize();

    CarPosition myCar(&self);
    Point me = myCar.location + myCar.direction() * (game.getCarWidth() / 2);
    double bestDist = 1e100;
    Tile result;
    for (unsigned long i = 0; i < map.width; i++) {
        for (unsigned long j = 0; j < map.height; j++) {
            double curDist = abs(me.x - (i + 0.5) * tileSize) + abs(me.y - (j + 0.5) * tileSize);
            if (curDist < bestDist) {
                bestDist = curDist;
                result = Tile(i, j);
            }
        }
    }
    return result;
};

vector<Go> collectMoves(
        double engineFrom, double engineTo, double engineStep,
        double wheelFrom, double wheelTo, double wheelStep,
        bool brake
) {
    vector<Go> result;
    for (double e = engineFrom; e <= engineTo; e += engineStep) {
        for (double w = wheelFrom; w <= wheelTo; w += wheelStep) {
            for (int b = 0; b <= brake; b++) {
                result.emplace_back(e, w, b);
            }
        }
    }
    return result;
}

Go experimentalBruteForce(const World& world, const Point& nextWaypoint) {
    Go best(-1.0, 0.0);
    double bestDist = 1e100;
    auto startState = State(&world);
    auto currentHealth = startState.myCar().health;

    auto firstMoves = collectMoves(0.0, 1.0, 0.5, -1.0, 1.0, 0.25, true);
    auto secondMoves = collectMoves(0.0, 1.0, 0.5, 0.0, 0.0, 0.25, false);

    for (auto& firstMove : firstMoves) {
        auto state = State(startState);
        for (int i = 0; i < 10; i++) {
            state.apply(firstMove);
            if (currentHealth - state.myCar().health > 0.03) break;
        }
        if (currentHealth - state.myCar().health > 0.03) continue;

        for (auto& secondMove : secondMoves) {
            auto next = State(state);
            for (int i = 0; i < 20; i++) {
                next.apply(secondMove);
                if (currentHealth - next.myCar().health > 0.03) break;
                double curDist = next.myCar().location.distanceTo(nextWaypoint);
                if (curDist < bestDist) {
                    bestDist = curDist;
                    best = firstMove;
                }
            }
        }
    }

    if (bestDist == 1e100) {
        cerr << "no best move on tick " << world.getTick() << endl;
    }

    return best;
}

Point computeNextWaypoint(const Car& self, const World& world, const Game& game) {
    auto curTile = findCurrentTile(self);
    auto& waypoints = world.getWaypoints();
    auto nextWaypoint = Tile(self.getNextWaypointX(), self.getNextWaypointY());
    auto path = bestPath(curTile, nextWaypoint);
    if (path.size() >= 2) return path[1].toPoint();

    unsigned long j = self.getNextWaypointIndex();
    j = (j + 1) % waypoints.size();
    auto nextNextWaypoint = Tile(waypoints[j][0], waypoints[j][1]);
    path = bestPath(nextWaypoint, nextNextWaypoint);
    return path[0].toPoint();
}

void drawWaypoints(const World& world) {
    auto& game = Const::getGame();

    auto& waypoints = world.getWaypoints();
    for (unsigned long i = 0, size = waypoints.size(); i < size; i++) {
        vis->drawText(Point(
                waypoints[i][0] * game.getTrackTileSize() + 20.0,
                waypoints[i][1] * game.getTrackTileSize() + 120.0
        ), string("#") + to_string(i));
    }
}

void printMove(const Move& move) {
    ostringstream ss;
    ss.precision(3);
    ss << fixed << move.getEnginePower() << "|" << move.getWheelTurn();
    if (move.isBrake()) ss << "|BRAKE";
    if (move.isThrowProjectile()) ss << "|FIRE";
    if (move.isUseNitro()) ss << "|NITRO";
    if (move.isSpillOil()) ss << "|OIL";
    
    vis->drawTextStatic(Point(450, 40), ss.str());
}

void MyStrategy::move(const Car& self, const World& world, const Game& game, Move& move) {
    static bool initialized = false;
    if (!initialized) {
        initialized = true;
        initialize(game);
    }

    auto& map = Map::getMap();
    map.update(world);

    drawWaypoints(world);

#ifdef DEBUG_PHYSICS_PREDICTION
    moveDebugPhysicsPrediction(self, world, game, move);
    return;
#endif

    if (world.getTick() < game.getInitialFreezeDurationTicks()) {
        move.setEnginePower(1.0);
        return;
    }

    Point nextWaypoint = computeNextWaypoint(self, world, game);

    vis->drawCircle(nextWaypoint, 100);
    vis->drawCircle(nextWaypoint, 150);

    Go best = experimentalBruteForce(world, nextWaypoint);
    best.applyTo(move);

    printMove(move);
}

MyStrategy::MyStrategy() { }
