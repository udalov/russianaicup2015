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
// #define NO_REVERSE_MODE

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
    FULL_SPEED_TURN_ON_DEFAULT_MAP,
};

const DPPMode mode = DPPMode::FULL_SPEED_TURN_ON_DEFAULT_MAP;

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
    } else if (mode == DPPMode::FULL_SPEED_TURN_ON_DEFAULT_MAP) {
        if (400 <= tick && tick <= 450) {
            return Go(-1.0, 1.0, 405 <= tick && tick <= 430);
        }
        return Go(1.0, 0.0);
    } else {
        terminate();
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
    } else if (mode == DPPMode::FULL_SPEED_TURN_ON_DEFAULT_MAP) {
        return false;
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

        auto actual = currentState.me();
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

    expectedPosByTick.insert({ world.getTick() + lookahead, state.me() });

    vis->drawRect(state.me().rectangle());
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

Point bumperCenter(const CarPosition& car) {
    static const double carWidth = Const::getGame().getCarWidth();

    return car.location + car.direction() * (carWidth / 2);
}

Tile findCurrentTile(const CarPosition& car) {
    static const double tileSize = Const::getGame().getTrackTileSize();

    Point bc = bumperCenter(car);
    return Tile(static_cast<int>(bc.x / tileSize), static_cast<int>(bc.y / tileSize));
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

double score(const State& startState, const State& state, const vector<Tile>& path, const vector<Segment>& pathSegment, bool debug = false) {
    auto& me = state.me();

    auto location = bumperCenter(me);

    unsigned long pathIndex = find(path.begin(), path.end(), findCurrentTile(startState.me())) - path.begin() + 1;
    if (findCurrentTile(me) == path[pathIndex] && pathIndex + 1 < path.size()) pathIndex++;

    /*
    if (state.original->getTick() >= 700) {
        cout << state.original->getTick();
        for (auto& tile : path) cout << " " << tile.toString();
        cout << endl << "  start " << findCurrentTile(startState.me()).toString() << " cur " << findCurrentTile(me).toString() << " " << pathIndex << endl;
    }
    */

    auto& next = pathSegment[pathIndex - 1];
    auto cur =
            pathIndex >= 2 ? pathSegment[pathIndex - 2] :
            next + Vec(next.center(), path.front().toPoint()) * 2.0;
    auto& nextNext = pathSegment[pathIndex];
    auto& nextNextNext = pathSegment[pathIndex + 1];

    auto curHealthDelta = min(me.health - startState.me().health + 0.03, 0.0);

    double result = curHealthDelta * 1e9;

    auto centerLine = Vec(cur.center(), next.center());
    {
        double travelled = pathIndex + Vec(cur.center(), location).projection(centerLine) / centerLine.length();
        result += travelled;
    }

    /*
    {
        auto point = next.center();
        auto nextNextCenter = nextNext.center();
        auto d1 = next.p1.distanceTo(nextNextCenter);
        auto d2 = next.p2.distanceTo(nextNextCenter);
        if (abs(d1 - d2) > 1e-2) {
            auto curCenter = cur.center();
            if (abs(next.p1.distanceTo(curCenter) - next.p2.distanceTo(curCenter)) > 1e-2) {
                point = d1 < d2 ? next.p1 : next.p2;
            } else {
                point = d1 > d2 ? next.p1 : next.p2;
            }
        }

        result += pathIndex * 5e6 - point.distanceTo(me.location) * 1e3;
    }
    */

    auto dir = me.direction();
    /*
    if (!isFacingTowards(me, next)) {
        auto minAngle = min(
                abs(dir.angleTo(Vec(location, next.p1))),
                abs(dir.angleTo(Vec(location, next.p2)))
        );
        result -= minAngle * 500.0;
    }
    */

    /*
    auto nextTurnAngle = abs(dir.angleTo(Vec(next.center(), nextNextNext.center())));
    result -= nextTurnAngle * 1400.0;
    */

    result += (me.velocity.projection(centerLine) / centerLine.length()) * 0.1;

    /*
    Point center;
    bool turn = false;
    if (cur.p1.y == cur.p2.y && next.p1.x == next.p2.x) {
        turn = true;
        center = Point(next.p1.x, cur.p1.y);
    } else if (cur.p1.x == cur.p2.x && next.p1.y == next.p2.y) {
        turn = true;
        center = Point(cur.p1.x, next.p1.y);
    }

    double turnAngle;
    if (turn) {
        auto angle = Vec(center, next.center()).angleTo(Vec(center, cur.center()));
        if (debug) cout << "  angle " << angle;
        auto v = Vec(center, location).rotate(angle);
        turnAngle = abs(dir.angleTo(v));
    } else {
        turnAngle = abs(dir.angleTo(Vec(cur.center(), next.center())));
    }

    if (debug) cout << "  turn-angle " << turnAngle << endl;

    result -= turnAngle * 0.1;
    */

    /*
    unsigned long j = pathIndex - 1;
    while (j + 1 < pathSegment.size() &&
           abs(pathSegment[j].p1.distanceTo(pathSegment[j + 1].center()) - pathSegment[j].p2.distanceTo(pathSegment[j + 1].center())) < 1e-2) {
        j++;
    }
    Point far = pathSegment[j].p1.distanceTo(pathSegment[j + 1].center()) > pathSegment[j].p2.distanceTo(pathSegment[j + 1].center())
        ? pathSegment[j].p1 : pathSegment[j].p2;
    Point g = cur.p1.distanceTo(far) < cur.p2.distanceTo(far) ? cur.p1 : cur.p2;
    auto dist = Line(far, g).distanceFrom(me.location);

    result -= dist * 0.5;
    */

    /*
    double angleDiff = Vec(cur.center(), next.center()).angleTo(me.direction());
    result -= abs(angleDiff) * 1000.0;
    */

    return result;
}

Go experimentalBruteForce(const World& world, const vector<Tile>& path) {
    Go best(-1.0, 0.0);
    Go bestSecondMove(best);
    Go bestThirdMove(best);
    double bestScore = -1e100;
    auto startState = State(&world);

    const int firstDuration = 10;
    const int secondDuration = 40;
    const int thirdDuration = 80;

    auto firstMoves = collectMoves(1.0, 1.0, 1.0, -1.0, 1.0, 0.125, true);
    for (auto brake : { false, true }) {
        firstMoves.emplace_back(startState.me().enginePower, startState.me().wheelTurn, brake);
    }
    auto secondMovesBase = collectMoves(1.0, 1.0, 1.0, -1.0, 1.0, 1.0, true);
    auto thirdMovesBase = collectMoves(1.0, 1.0, 1.0, 0.0, 0.0, 1.0, true);

    // TODO: more careful filtering
    for (int moveNumber = 1; moveNumber <= 2; moveNumber++) {
        auto& v = moveNumber == 1 ? firstMoves : secondMovesBase;
        for (unsigned long i = 0; i < v.size(); i++) {
            auto& go = v[i];
            bool remove = false;
            if (go.enginePower < 0.0 && go.wheelTurn != -1.0 && go.wheelTurn != 0.0 && go.wheelTurn != 1.0) {
                remove = true;
            }
            if (moveNumber == 1) {
                if (go.enginePower != -1.0 && go.enginePower != 1.0 &&
                    abs(startState.me().enginePower - go.enginePower) > firstDuration * Const::getGame().getCarEnginePowerChangePerTick()) {
                    remove = true;
                }
                if (go.wheelTurn != -1.0 && go.wheelTurn != 1.0 &&
                    abs(startState.me().wheelTurn - go.wheelTurn) > firstDuration * Const::getGame().getCarWheelTurnChangePerTick()) {
                    remove = true;
                }
            }
            if (remove) {
                v.erase(v.begin() + i);
                i--;
            }
        }
    }
    
    // if (world.getTick() >= 700) cout << "moves " << firstMoves.size() << " " << secondMovesBase.size() << " " << thirdMovesBase.size() << endl;

    vector<Segment> pathSegment;
    for (unsigned long i = 0, size = path.size(); i < size - 1; i++) {
        pathSegment.push_back(segmentBetweenTiles(path[i], path[i + 1]));
    }

    vis->drawLine(pathSegment.front().p1, pathSegment.front().p2);

    for (auto& firstMove : firstMoves) {
        auto afterFirst = State(startState);
        for (int i = 0; i < firstDuration; i++) {
            afterFirst.apply(firstMove);
        }

        double firstScore = score(startState, afterFirst, path, pathSegment);

        auto secondMoves = secondMovesBase;
        if (find(secondMoves.begin(), secondMoves.end(), firstMove) == secondMoves.end()) {
            secondMoves.push_back(firstMove);
        }

        for (auto& secondMove : secondMoves) {
            auto afterSecond = State(afterFirst);
            for (int j = 0; j < secondDuration; j++) {
                afterSecond.apply(secondMove);
            }

            double secondScore = score(startState, afterSecond, path, pathSegment);

            auto thirdMoves = thirdMovesBase;
            if (find(thirdMoves.begin(), thirdMoves.end(), firstMove) == thirdMoves.end()) {
                thirdMoves.push_back(firstMove);
            }
            if (find(thirdMoves.begin(), thirdMoves.end(), secondMove) == thirdMoves.end()) {
                thirdMoves.push_back(secondMove);
            }

            for (auto& thirdMove : thirdMoves) {
                auto afterThird = State(afterSecond);
                for (int k = 0; k < thirdDuration; k++) {
                    afterThird.apply(thirdMove);
                }

                double curScore = score(startState, afterThird, path, pathSegment) + secondScore * 4.0;
                if (curScore > bestScore) {
                    bestScore = curScore;
                    best = firstMove;
                    bestSecondMove = secondMove;
                    bestThirdMove = thirdMove;
                }
            }
        }
    }

    {
        auto state = State(startState);
        for (int i = 0; i < firstDuration; i++) {
            state.apply(best);
        }
        for (int i = 0; i < secondDuration; i++) {
            state.apply(bestSecondMove);
        }
        for (int i = 0; i < thirdDuration; i++) {
            state.apply(bestThirdMove);
        }
        vis->drawRect(state.me().rectangle());
        cout << "tick " << world.getTick() << " best-score " << bestScore <<
            " #1 " << best.toString() << " #2 " << bestSecondMove.toString() << " #3 " << bestThirdMove.toString() << endl;
        score(startState, state, path, pathSegment, true);
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
#ifdef NO_REVERSE_MODE
    return false;
#else
    static int lastNonZeroSpeedTick = Const::getGame().getInitialFreezeDurationTicks();
    static int reverseUntilTick = -1;
    static int waitUntilTick = -1;

    auto tick = world.getTick();
    auto nonZeroSpeed = abs(me.velocity.length()) > 1.0;

    // cout << "tick " << tick << " last-non-zero " << lastNonZeroSpeedTick << " reverse-until " << reverseUntilTick << " wait-until " << waitUntilTick << " non-zero " << nonZeroSpeed << endl;

    if (tick <= reverseUntilTick) {
        result = Go(-1.0, 0.0);
        return true;
    }

    if (nonZeroSpeed || tick <= reverseUntilTick || tick <= waitUntilTick) {
        lastNonZeroSpeedTick = tick;
    }

    if (tick > waitUntilTick && tick - lastNonZeroSpeedTick >= 20) {
        reverseUntilTick = tick + 50;
        waitUntilTick = tick + 100;
    }

    return false;
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

    if (self.isFinishedTrack()) return;

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

#ifdef VISUALIZE
    printMove(move);
#endif
}

MyStrategy::MyStrategy() { }
