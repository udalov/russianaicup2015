#include "math2d.h"
#include "MyStrategy.h"
#include "Const.h"
#include "Map.h"
#include "Path.h"
#include "State.h"
#include "VisClient.h"

#include <algorithm>
#include <iostream>
#include <sstream>
#include <unordered_map>

using namespace model;
using namespace std;

// #define DEBUG_PHYSICS_PREDICTION
// #define REVERSE_MODE

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

Tile findCurrentTile(const CarPosition& car) {
    static const double tileSize = Const::getGame().getTrackTileSize();
    static const double carWidth = Const::getGame().getCarWidth();

    Point me = car.location + car.direction() * (carWidth / 2);
    return Tile((int)(me.x / tileSize), (int)(me.y / tileSize));
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
                result.emplace_back(e, w, (bool) b);
            }
        }
    }
    return result;
}

Segment segmentBetweenTiles(const Tile& tile1, const Tile& tile2) {
    static const double angle = M_PI / 6;
    static const double coeff = 0.5 / myCos(angle);
    auto p1 = tile1.toPoint();
    auto p2 = tile2.toPoint();
    auto v = Vec(p1, p2) * coeff;
    return Segment(p1 + v.rotate(angle), p1 + v.rotate(-angle));
}

bool isFacingTowards(const CarPosition& car, const Segment& segment) {
    auto& p = car.location;
    return (Vec(p, segment.p1) ^ car.velocity) < 0 && (car.velocity ^ Vec(p, segment.p2)) < 0;
}

Go experimentalBruteForce(const World& world, const vector<Tile>& path) {
    Go best(-1.0, 0.0);
    Go bestSecondMove(best);
    double bestScore = -1e100;
    auto startState = State(&world);
    auto startHealth = startState.myCar().health;

    auto firstMoves = collectMoves(-1.0, 1.0, 0.5, -1.0, 1.0, 0.25, true);
    auto secondMovesBase = collectMoves(0.0, 1.0, 0.5, 0.0, 0.0, 0.25, false);

    vector<Segment> pathSegment;
    for (unsigned long i = 0, size = path.size(); i < size - 1; i++) {
        pathSegment.push_back(segmentBetweenTiles(path[i], path[i + 1]));
    }

    vis->drawLine(pathSegment.front().p1, pathSegment.front().p2);

    for (auto& firstMove : firstMoves) {
        auto state = State(startState);
        for (int i = 0; i < 15; i++) {
            state.apply(firstMove);
        }

        auto secondMoves = secondMovesBase;
        secondMoves.push_back(firstMove);
        for (auto& secondMove : secondMoves) {
            auto next = State(state);
            for (int j = 0; j < 30; j++) {
                next.apply(secondMove);
            }

            auto& me = next.myCar();

            auto curTile = findCurrentTile(me);
            unsigned long pathIndex = find(path.begin(), path.end(), findCurrentTile(state.myCar())) - path.begin() + 1;
            if (curTile == path[pathIndex] && pathIndex + 1 < path.size()) pathIndex++;
            auto& nextSegment = pathSegment[pathIndex - 1];

            auto curHealthDelta = min(me.health - startHealth + 0.03, 0.0);

            /*
            auto curCenter = pathIndex >= 2 ? pathSegment[pathIndex - 2].center() : path.front().toPoint();
            auto nextCenter = nextSegment.center();
            auto centerLine = Vec(curCenter, nextCenter);
            auto travelled = pathIndex + Vec(curCenter, me.location).projection(centerLine) / centerLine.length();
            */

            double travelled;
            {
                auto point = nextSegment.center();
                auto nextNextCenter = pathSegment[pathIndex].center();
                auto d1 = nextSegment.p1.distanceTo(nextNextCenter);
                auto d2 = nextSegment.p2.distanceTo(nextNextCenter);
                bool nearest = false;
                if (pathIndex >= 2) {
                    auto curCenter = pathSegment[pathIndex - 2].center();
                    if (abs(nextSegment.p1.distanceTo(curCenter) - nextSegment.p2.distanceTo(curCenter)) > 1e-2) {
                        nearest = true;
                    }
                }
                if (abs(d1 - d2) > 1e-2) {
                    point = ((d1 < d2) == nearest) ? nextSegment.p1 : nextSegment.p2;
                }

                travelled = pathIndex * 1e3 - point.distanceTo(me.location);
            }

            double curScore = curHealthDelta * 1e9 + travelled * 1e3;

            if (!isFacingTowards(me, nextSegment)) {
                auto dir = me.direction();
                auto minAngle = min(
                        abs(dir.angleTo(Vec(me.location, nextSegment.p1))),
                        abs(dir.angleTo(Vec(me.location, nextSegment.p2)))
                );
                curScore -= minAngle * 50.0;
            }

            curScore += me.velocity.projection(Vec(pathSegment[pathIndex - 2].center(), nextSegment.center())) * 40.0;

            /*
            double angleDiff = Vec(pathSegment[pathIndex - 2].center(), nextSegment.center()).angleTo(me.direction());
            curScore -= abs(angleDiff) * 1000.0;
            */

            if (curScore > bestScore) {
                bestScore = curScore;
                best = firstMove;
                bestSecondMove = secondMove;
            }
        }
    }

    {
        auto state = State(startState);
        for (int i = 0; i < 15; i++) {
            state.apply(best);
        }
        for (int i = 0; i < 20; i++) {
            state.apply(bestSecondMove);
        }
        vis->drawRect(state.myCar().rectangle());
        cout << "tick " << world.getTick() << " best-score " << bestScore << " #1 " << best.toString() << " #2 " << bestSecondMove.toString() << endl;
    }

    return best;
}

vector<Tile> computePath(const Car& self, const World& world, const Game& game) {
    auto& waypoints = world.getWaypoints();
    int i = self.getNextWaypointIndex();
    Tile start = findCurrentTile(CarPosition(&self));
    vector<Tile> result;
    result.push_back(start);
    for (int steps = 0; steps < 5; steps++) {
        Tile finish = Tile(waypoints[i][0], waypoints[i][1]);
        auto path = bestPath(start, finish);
        result.insert(result.end(), path.begin() + 1, path.end());
        start = finish;
        i = (i + 1) % waypoints.size();
    }
    return result;
}

bool reverseMode(const CarPosition& me, const World& world, Go& result) {
#ifndef REVERSE_MODE
    return false;
#else
    static int lastZeroSpeedTick = -1;
    static int reverseUntilTick = -1;

    auto tick = world.getTick();

    // TODO: check this method

    if (reverseUntilTick != -1 && tick <= reverseUntilTick) {
        result = Go(-1.0, 0.0);
        return true;
    }

    if (abs(me.velocity.length()) > 1e-1) {
        lastZeroSpeedTick = -1;
        return false;
    }

    if (lastZeroSpeedTick == -1) lastZeroSpeedTick = tick;
    if (tick - lastZeroSpeedTick < 20) return false;

    reverseUntilTick = tick + 40;
    result = Go(-1.0, 0.0);
    return true;
#endif
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

#ifdef DEBUG_PHYSICS_PREDICTION
    moveDebugPhysicsPrediction(self, world, game, move);
    return;
#endif

    if (world.getTick() < game.getInitialFreezeDurationTicks()) {
        move.setEnginePower(1.0);
        return;
    }

    Go reverse;
    if (reverseMode(CarPosition(&self), world, reverse)) {
        reverse.applyTo(move);
        return;
    }

    auto path = computePath(self, world, game);

    Go best = experimentalBruteForce(world, path);
    best.applyTo(move);

    printMove(move);
}

MyStrategy::MyStrategy() { }
