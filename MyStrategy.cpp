#include "Const.h"
#include "Map.h"
#include "MyStrategy.h"
#include "State.h"
#include "VisClient.h"

#include <iostream>
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

    vis->drawRect(expectedMyCar.rectangle());

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

void drawMap() {
    auto g = Map::getMap().graph;
    const int dx[] = {1, 0, -1, 0};
    const int dy[] = {0, 1, 0, -1};
    const double s = 200.0;
    for (auto i = 0; i < g.size(); i++) {
        for (auto j = 0; j < g[i].size(); j++) {
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

pair<int, int> findCurrentTile(const Car& self) {
    auto& map = Map::getMap();
    auto& game = Const::getGame();
    const double tileSize = game.getTrackTileSize();

    CarPosition myCar(&self);
    Point me = myCar.location + myCar.direction() * (game.getCarHeight() / 2);
    double bestDist = 1e100;
    pair<int, int> result;
    for (int i = 0; i < map.width; i++) {
        for (int j = 0; j < map.height; j++) {
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
    
    vector<int> q;
    unordered_map<int, int> prev;
    int start = (x << 8) + y;
    q.push_back(start);
    prev[start] = -1;
    int qb = 0;
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
    Go best(-1.0, 0.0);
    double bestDist = 1e100;
    auto startState = State(&world);
    for (double e1 = 0.0; e1 <= 1.0; e1 += 0.5) {
        for (double w1 = -1.0; w1 <= 1.0; w1 += 0.25) {
            for (bool brake : { false, true }) {
                Go go(e1, w1, brake);
                vector<Go> moves = { go, go, go, go };
                auto state = startState;
                for (int i = 0; i < 10; i++) {
                    state = state.apply(moves);
                    if (state.cars.front().health == 0.0) break;
                }
                if (state.cars.front().health == 0.0) continue;

                for (double e2 = 0.0; e2 <= 1.0; e2 += 0.5) {
                    Go future(e2, 0.0);
                    vector<Go> futureMoves = { future, future, future, future };
                    auto next = State(state);
                    for (int i = 0; i < 20; i++) {
                        next = next.apply(futureMoves);
                        auto myFutureCar = next.cars.front();
                        if (myFutureCar.health == 0.0) break;
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

Point computeNextWaypoint(const Car& self, const Game& game) {
    auto curTile = findCurrentTile(self);
    auto nextTile =
            nextTileToReachWaypoint(curTile.first, curTile.second, self.getNextWaypointX(), self.getNextWaypointY());
    nextTile =
            nextTileToReachWaypoint(nextTile.first, nextTile.second, self.getNextWaypointX(), self.getNextWaypointY());
    return Point(
            (nextTile.first + 0.5) * game.getTrackTileSize(),
            (nextTile.second + 0.5) * game.getTrackTileSize()
    );
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

    Point nextWaypoint = computeNextWaypoint(self, game);

    vis->drawCircle(nextWaypoint, 100);
    vis->drawCircle(nextWaypoint, 150);

    Go best = experimentalBruteForce(world, nextWaypoint);
    move.setEnginePower(best.enginePower);
    move.setWheelTurn(best.wheelTurn);
}

MyStrategy::MyStrategy() { }
