#include "math2d.h"
#include "MyStrategy.h"
#include "Const.h"
#include "Debug.h"
#include "Map.h"
#include "Path.h"
#include "Scorer.h"
#include "State.h"
#include "Tracks.h"
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
        state.apply(go);
    }

    expectedPosByTick.insert({ world.getTick() + lookahead, state.me() });

    vis->drawRect(state.me().rectangle);
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

// TODO: only use them if they are from the previous tick
vector<Track> previousTracks;

Go solve(const World& world, const vector<Tile>& path) {
    auto startState = State(&world);

    // TODO: constant
    auto tracks = vector<Track>(previousTracks.begin(), previousTracks.begin() + min(previousTracks.size(),
            static_cast<vector<Track>::size_type>(30)
    ));
    collectTracks(startState.me(), tracks);
    auto scorer = Scorer(startState, path, vis);

    for (auto& track : tracks) {
        double score = 0.0;

        auto state = State(startState);
        for (unsigned long i = 0, size = track.moves.size(); i < size; i++) {
            state.apply(track.moves[i]);

            const int firstScoreTurn = 40;
            const int stepScoreTurn = 5;
            if (i >= firstScoreTurn && !(i % stepScoreTurn)) {
                double s = scorer.score(state);
                score += s / ((i - firstScoreTurn + stepScoreTurn) / stepScoreTurn);
            }
        }

        track.setScore(score);
    }

    sort(tracks.rbegin(), tracks.rend());

    auto& bestTrack = tracks.front();
    auto bestMove = bestTrack.moves.front();

    {
        auto state = State(startState);
        for (auto& go : bestTrack.moves) {
            state.apply(go);
        }
        vis->drawRect(state.me().rectangle);
        cout << "tick " << world.getTick() << " tracks " << tracks.size() << " best-score " << bestTrack.score << " best-move " << bestMove.toString() << endl;
    }

    previousTracks.resize(tracks.size());
    transform(tracks.begin(), tracks.end(), previousTracks.begin(), [](const Track& track) {
        return track.drop(1);
    });

    return bestMove;
}

vector<Tile> computePath(const Car& self, const World& world, const Game& game) {
    auto& waypoints = world.getWaypoints();
    int i = self.getNextWaypointIndex();
    Tile start = CarPosition(&self).tile();
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
        result = Go(-1.0, 0.0, me.enginePower > 0.0);
        return true;
    }

    if (nonZeroSpeed || tick <= reverseUntilTick || tick <= waitUntilTick) {
        lastNonZeroSpeedTick = tick;
    }

    if (tick > waitUntilTick && tick - lastNonZeroSpeedTick >= 20) {
        reverseUntilTick = tick + 120;
        waitUntilTick = tick + 180;
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

    Go solution = solve(world, path);
    solution.applyTo(move);

#ifdef VISUALIZE
    printMove(move);
#endif
}

MyStrategy::MyStrategy() { }
