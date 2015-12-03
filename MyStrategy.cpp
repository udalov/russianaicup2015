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

#define TURN_LEFT WheelTurnDirection::TURN_LEFT
#define KEEP WheelTurnDirection::KEEP
#define TURN_RIGHT WheelTurnDirection::TURN_RIGHT

// #define DEBUG_PHYSICS_PREDICTION

#ifndef ONLINE_JUDGE
#define VISUALIZE
#define DEBUG_OUTPUT
#endif

const int NITRO_CHECK_PERIOD = 10;

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
            return Go(1.0, TURN_RIGHT, false, false, true);
        }
        if (280 <= tick && tick < 285) {
            return Go(1.0, TURN_RIGHT);
        }
        return Go(1.0, KEEP);
    } else if (mode == DPPMode::COLLISION_WITH_SEGMENT_SIDEWAYS) {
        if (400 <= tick && tick < 450) {
            return Go(-1.0, TURN_RIGHT);
        }
        return Go(1.0, KEEP);
    } else if (mode == DPPMode::COLLISION_WITH_CORNER) {
        if (375 <= tick && tick < 450) {
            return Go(-1.0, TURN_RIGHT);
        }
        if (450 <= tick && tick < 480) {
            return Go(-1.0, KEEP);
        }
        return Go(1.0, KEEP);
    } else if (mode == DPPMode::BRAKE) {
        if (250 <= tick && tick <= 260) {
            return Go(1.0, KEEP, true);
        }
        return Go(1.0, KEEP);
    } else if (mode == DPPMode::FULL_SPEED_TURN_ON_DEFAULT_MAP) {
        if (400 <= tick && tick <= 450) {
            return Go(-1.0, TURN_RIGHT, 405 <= tick && tick <= 430);
        }
        return Go(1.0, KEEP);
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
    experimentalMove.applyTo(self, move);

    auto currentState = State(&world);

    if (isPhysicsPredictionOutputNeeded(world.getTick())) {
        const double eps = 1e-9;
        auto currentInfo = expectedPosByTick.find(world.getTick());
        if (currentInfo == expectedPosByTick.end()) return;

        auto actual = currentState.me();
        auto& predicted = currentInfo->second;
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
        auto diffProjectiles = actual.projectiles - predicted.projectiles;
        if (abs(diffProjectiles) > eps) cout << " ammo " << diffProjectiles;
        auto diffNitroCharges = actual.nitroCharges - predicted.nitroCharges;
        if (abs(diffNitroCharges) > eps) cout << " nitros " << diffNitroCharges;
        auto diffOilCanisters = actual.oilCanisters - predicted.oilCanisters;
        if (abs(diffOilCanisters) > eps) cout << " oil " << diffOilCanisters;
        auto diffNitroCooldown = actual.nitroCooldown - predicted.nitroCooldown;
        if (abs(diffNitroCooldown) > eps) cout << " nitro-cooldown " << diffNitroCooldown;
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
    auto& map = Map::getMap();
    const int dx[] = {1, 0, -1, 0};
    const int dy[] = {0, 1, 0, -1};
    const double s = 200.0;
    for (unsigned long i = 0; i < map.width; i++) {
        for (unsigned long j = 0; j < map.height; j++) {
            double x = i * s + s/2;
            double y = j * s + s/2;
            for (int d = 0; d < 4; d++) {
                if (map.get(i, j) & (1 << d)) {
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
            static_cast<vector<Track>::size_type>(50)
    ));
    collectTracks(startState.me(), tracks);
    if (tracks.empty()) return Go();

    if (!(world.getTick() % NITRO_CHECK_PERIOD) && startState.me().nitroCharges > 0 && startState.me().nitroCooldown == 0) {
        for (unsigned long i = 0, size = tracks.size(); i < size; i++) {
            auto copy = Track(tracks[i]);
            if (copy.moves().empty()) continue;
            copy.moves().front().useNitro = true;
            tracks.push_back(copy);
        }
    }

    auto scorer = Scorer(startState, path, vis);

    for (auto& track : tracks) {
        track.setScore(scorer.scoreTrack(track));
    }

    sort(tracks.rbegin(), tracks.rend());

    auto& bestTrack = tracks.front();
    auto bestMove = bestTrack.moves().empty() ? Go() : bestTrack.moves().front();

    {
        // Debug::debug = true;
        auto state = State(startState);
        for (auto& go : bestTrack.moves()) {
            state.apply(go);
            cout << "  " << go.toString() << endl;
            // Debug::debug = false;
            vis->drawRect(state.me().rectangle);
        }
        vis->drawRect(state.me().rectangle);
#ifdef DEBUG_OUTPUT
        cout << "tick " << world.getTick() << " tracks " << tracks.size() << " best-score " << bestTrack.score() <<
            " best-move " << bestMove.toString() << " " << state.me().toString() << endl;
#endif
        scorer.scoreState(state, true);
    }

    previousTracks.resize(tracks.size());
    transform(tracks.begin(), tracks.end(), previousTracks.begin(), [](const Track& track) {
        return track.rotate();
    });

    return bestMove;
}

vector<Tile> computePath(const Car& self, const World& world, const Game& game) {
    auto& waypoints = world.getWaypoints();
    int i = self.getNextWaypointIndex();
    DirectedTile start = CarPosition(&self).directedTile();
    vector<Tile> result;
    result.push_back(start.tile);
    for (int steps = 0; steps < 5; steps++) {
        Tile finish = Tile(waypoints[i][0], waypoints[i][1]);
        auto path = bestPath(start, finish);
        // cout << "  path from " << start.toString() << " to " << finish.toString() << ":"; for (auto& p : path) cout << " " << p.toString(); cout << endl;
        for (auto it = path.begin() + 1; it != path.end(); ++it) {
            if (it->tile != result.back()) {
                result.push_back(it->tile);
            }
        }
        start = path.back();
        i = (i + 1) % waypoints.size();
    }
    return result;
}

bool safeModeForward(const World& world, Go& result) {
    // Try going forward for the next epsilon ticks and see if it helps
    auto startState = State(&world);
    vector<Track> tracks;
    for (auto wheel : { TURN_LEFT, KEEP, TURN_RIGHT }) {
        tracks.emplace_back(vector<Go>(25, Go(1.0, wheel)));
    }
    for (auto& track : tracks) {
        auto state = State(startState);
        for (auto& move : track.moves()) {
            state.apply(move);
        }
        if (state.me().velocity.length() > 1.0) {
            result = track.moves().front();
            return true;
        }
    }
    return false;
}

bool safeMode(const CarPosition& me, const World& world, Move& move) {
    static int lastNonZeroSpeedTick = Const::getGame().getInitialFreezeDurationTicks();
    static int safeUntilTick = -1;
    static int waitUntilTick = -1;

    auto tick = world.getTick();
    auto nonZeroSpeed = abs(me.velocity.length()) > 1.0;

    // cout << "tick " << tick << " last-non-zero " << lastNonZeroSpeedTick << " safe-until " << safeUntilTick << " wait-until " << waitUntilTick << " non-zero " << nonZeroSpeed << endl;

    if (tick <= safeUntilTick) {
        /*
        Go forward;
        if (safeModeForward(world, forward)) {
            forward.applyTo(*me.original, move);
            return true;
        }
        */
        move.setEnginePower(-1.0);
        move.setWheelTurn(0.0);
        if (me.enginePower > 0.0) {
            move.setBrake(true);
        }
        return true;
    }

    if (nonZeroSpeed || tick <= safeUntilTick || tick <= waitUntilTick) {
        lastNonZeroSpeedTick = tick;
    }
    
    if (tick <= waitUntilTick || tick - lastNonZeroSpeedTick < 20) {
        return false;
    }

    safeUntilTick = tick + 120;
    waitUntilTick = tick + 180;

    /*
    Go forward;
    if (safeModeForward(world, forward)) {
        forward.applyTo(*me.original, move);
        return true;
    }
    */

    return false;
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

    if (safeMode(CarPosition(&self), world, move)) {
        return;
    }

    auto path = computePath(self, world, game);

    Go solution = solve(world, path);
    solution.applyTo(self, move);

#ifdef VISUALIZE
    printMove(move);
#endif
}

MyStrategy::MyStrategy() { }
