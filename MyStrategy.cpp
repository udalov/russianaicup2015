#include "math2d.h"
#include "Const.h"
#include "Map.h"
#include "MyStrategy.h"
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
        auto diffHealth = actual.health - predicted.health;
        if (abs(diffHealth) > eps) cout << " health " << diffHealth;
        auto diffNitroCharges = actual.nitroCharges - predicted.nitroCharges;
        if (abs(diffNitroCharges) > eps) cout << " nitros " << diffNitroCharges;
        auto diffNitroCooldown = actual.nitroCooldown - predicted.nitroCooldown;
        if (abs(diffNitroCooldown) > eps) cout << " nitro cooldown " << diffNitroCooldown;
        cout << endl;
    }

    const int lookahead = 1;

    auto state = currentState;
    for (int i = 0; i < lookahead; i++) {
        auto go = goDebugPhysicsPrediction(world.getTick() + i);
        vector<Go> moves(world.getCars().size(), go);
        state = state.apply(moves);
    }

    auto expectedMyCar = state.getCarById(self.getId());
    expectedPosByTick.insert({ world.getTick() + lookahead, expectedMyCar });

    vis->drawRect(expectedMyCar.rectangle());
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

pair<int, int> findCurrentTile(const Car& self) {
    auto& map = Map::getMap();
    auto& game = Const::getGame();
    const double tileSize = game.getTrackTileSize();

    CarPosition myCar(&self);
    Point me = myCar.location + myCar.direction() * (game.getCarHeight() / 2);
    double bestDist = 1e100;
    pair<int, int> result;
    for (unsigned long i = 0; i < map.width; i++) {
        for (unsigned long j = 0; j < map.height; j++) {
            double curDist = abs(me.x - (i + 0.5) * tileSize) + abs(me.y - (j + 0.5) * tileSize);
            if (curDist < bestDist) {
                bestDist = curDist;
                result = make_pair(i, j);
            }
        }
    }
    return result;
};

pair<int, int> nextTileToReachWaypoint(int x, int y, int wx, int wy) {
    static const int dx[] = {1, 0, -1, 0};
    static const int dy[] = {0, 1, 0, -1};
    auto& map = Map::getMap();

    if (x == wx && y == wy) return make_pair(x, y);
    
    vector<int> q;
    unordered_map<int, int> prev;
    int start = (x << 8) + y;
    q.push_back(start);
    prev[start] = -1;
    unsigned long qb = 0;
    while (qb < q.size()) {
        int v = q[qb++];
        int xx = v >> 8, yy = v & 255;
        if (xx == wx && yy == wy) {
            while (prev.find(v) != prev.end()) {
                if (prev[v] == start) return make_pair(v >> 8, v & 255);
                v = prev[v];
            }
            cerr << "bad path" << endl;
            return make_pair(x, y);
        }
        for (int d = 0; d < 4; d++) {
            if (map.graph[xx][yy] & (1 << d)) {
                int nx = xx + dx[d], ny = yy + dy[d];
                int nv = (nx << 8) + ny;
                if (prev.find(nv) == prev.end()) {
                    prev[nv] = v;
                    q.push_back(nv);
                }
            }
        }
    }
    
    cerr << "path not found from " << x << " " << y << " to " << wx << " " << wy << endl;
    return make_pair(x, y);
}

Go experimentalBruteForce(const World& world, const Point& nextWaypoint) {
    static const unsigned long carsCount = world.getCars().size();

    Go best(-1.0, 0.0);
    double bestDist = 1e100;
    auto startState = State(&world);
    auto currentHealth = startState.cars.front().health;

    for (double e1 = 0.0; e1 <= 1.0; e1 += 0.5) {
        for (double w1 = -1.0; w1 <= 1.0; w1 += 0.25) {
            for (bool brake : { false, true }) {
                Go go(e1, w1, brake);
                vector<Go> moves(carsCount, go);
                auto state = startState;
                for (int i = 0; i < 10; i++) {
                    state = state.apply(moves);
                    if (currentHealth - state.cars.front().health > 0.03) break;
                }
                if (currentHealth - state.cars.front().health > 0.03) continue;

                for (double e2 = 0.0; e2 <= 1.0; e2 += 0.5) {
                    Go future(e2, 0.0);
                    vector<Go> futureMoves(carsCount, future);
                    auto next = State(state);
                    for (int i = 0; i < 20; i++) {
                        next = next.apply(futureMoves);
                        auto myFutureCar = next.cars.front();
                        if (currentHealth - myFutureCar.health > 0.03) break;
                        double curDist = myFutureCar.location.distanceTo(nextWaypoint);
                        if (curDist < bestDist) {
                            bestDist = curDist;
                            best = go;
                        }
                    }
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
    auto& waypoints = world.getWaypoints();
    for (unsigned long i = 0, size = waypoints.size(); i < size; i++) {
        vis->drawText(Point(
                waypoints[i][0] * game.getTrackTileSize() + 20.0,
                waypoints[i][1] * game.getTrackTileSize() + 120.0
        ), string("#") + to_string(i));
    }

    auto curTile = findCurrentTile(self);
    auto nextWaypoint = make_pair(self.getNextWaypointX(), self.getNextWaypointY());
    auto nextTile =
            nextTileToReachWaypoint(curTile.first, curTile.second, nextWaypoint.first, nextWaypoint.second);
    if (nextTile == nextWaypoint) {
        unsigned long j = self.getNextWaypointIndex();
        j = (j + 1) % waypoints.size();
        nextWaypoint = make_pair(waypoints[j][0], waypoints[j][1]);
    }
    nextTile =
            nextTileToReachWaypoint(nextTile.first, nextTile.second, nextWaypoint.first, nextWaypoint.second);
    return Point(
            (nextTile.first + 0.5) * game.getTrackTileSize(),
            (nextTile.second + 0.5) * game.getTrackTileSize()
    );
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

    Point nextWaypoint = computeNextWaypoint(self, world, game);

    vis->drawCircle(nextWaypoint, 100);
    vis->drawCircle(nextWaypoint, 150);

    Go best = experimentalBruteForce(world, nextWaypoint);
    best.applyTo(move);

    printMove(move);
}

MyStrategy::MyStrategy() { }
