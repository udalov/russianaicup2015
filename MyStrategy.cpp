#include "math2d.h"
#include "MyStrategy.h"
#include "Collider.h"
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
// #define VISUALIZE
#define DEBUG_OUTPUT
#endif

const int NITRO_CHECK_PERIOD = 10;
const int FIRE_CHECK_PERIOD = 4;
const int OIL_CHECK_PERIOD = 10;

VisClient *vis = nullptr;

void initialize(const Game& game) {
#ifdef VISUALIZE
    vis = new VisClient(29292);
#else
    vis = new VisClient(-1);
#endif
    vis->send("hello");

    Const::getInstance().game = game;
    cout.precision(3);
    cout << fixed;
    cerr.precision(3);
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

    auto currentState = State(&world, self.getTeammateIndex());

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

bool intersectsWallsHeuristic(const Point& point, double radius) {
    static const int sx[] = {1, -1, -1, 1};
    static const int sy[] = {1, 1, -1, -1};

    auto& map = Map::getMap();

    static const double trackTileSize = Const::getGame().getTrackTileSize();
    static const double trackTileMargin = Const::getGame().getTrackTileMargin();

    auto tile = Tile(point);
    auto center = tile.toPoint();

    for (int d = 0; d < 4; d++) {
        auto corner = Point(
                center.x + sx[d] * trackTileSize / 2.0,
                center.y + sy[d] * trackTileSize / 2.0
        );
        if (point.distanceTo(corner) < radius + trackTileMargin + 1e-6) return true;

        if (map.get(tile.x, tile.y) & (1 << d)) continue;

        auto previousCorner = Point(
                center.x + sx[(d + 3) & 3] * trackTileSize / 2.0,
                center.y + sy[(d + 3) & 3] * trackTileSize / 2.0
        );
        auto segment = Segment(corner, previousCorner);
        if (segment.distanceFrom(point) < radius + trackTileMargin + 1e-6) return true;
    }

    return false;
}

bool shouldFire(const State& startState) {
    static auto& game = Const::getGame();

    const unsigned long ticksLookahead = 30;
    const unsigned long ticksCooldown = 4;

    auto& me = startState.me();

    auto radius = me.isBuggy() ? game.getWasherRadius() : game.getTireRadius();
    auto initialSpeed = me.isBuggy() ? game.getWasherInitialSpeed() : game.getTireInitialSpeed();
    auto velocity = me.direction() * initialSpeed;

    vector<Point> projectile;
    auto location = me.location;
    for (unsigned long tick = 0; tick < ticksLookahead; tick++) {
        location += velocity;
        projectile.push_back(location);
        if (!me.isBuggy() && intersectsWallsHeuristic(location, radius)) {
            // cout << "  projectile from " << me.toString() << endl;
            // cout << "  will intersect the wall on tick " << tick << " at " << location.toString() << endl;
            break;
        }
    }

    auto distance = radius + game.getCarHeight() / 2;
    auto targetMove = Go(1.0, KEEP);

    // for (auto& t : projectile) { vis->drawCircle(t, distance); }

    bool hitsEnemy = false;

    for (auto& car : startState.original->getCars()) {
        if (car.getId() == me.original->getId()) continue;
        if (car.isFinishedTrack()) continue;
        if (abs(car.getDurability()) < 1e-9) continue;

        auto state = State(startState.original, car.getId());
        auto& target = state.me();
        for (unsigned long tick = 0; tick < projectile.size(); tick++) {
            state.apply(targetMove);
            if (target.location.distanceTo(projectile[tick]) < distance) {
                if (car.isTeammate()) return false;
                if (tick > ticksCooldown) {
                    hitsEnemy = true;
                    break;
                }
            }
        }
    }

    return hitsEnemy;
}

bool shouldSpillOil(const State& startState) {
    static auto& game = Const::getGame();
    static const double radius = game.getOilSlickRadius() * 0.8;
    static const double shift = game.getCarWidth() / 2 + game.getOilSlickInitialRange() + game.getOilSlickRadius();

    const unsigned long ticksLookahead = 70;
    const unsigned long ticksCooldown = 20;

    auto& me = startState.me();

    auto location = me.location - (me.direction() * shift);
    auto targetMove = Go(1.0, KEEP);

    // vis->drawCircle(location, radius);

    bool enemyCaught = false;

    for (auto& car : startState.original->getCars()) {
        if (car.getId() == me.original->getId()) continue;
        if (car.isFinishedTrack()) continue;
        if (abs(car.getDurability()) < 1e-9) continue;

        auto state = State(startState.original, car.getId());
        auto& target = state.me();
        for (unsigned long tick = 0; tick < ticksLookahead; tick++) {
            state.apply(targetMove);
            for (auto& point : target.rectangle.points()) {
                if (point.distanceTo(location) < radius) {
                    if (car.isTeammate()) return false;
                    if (tick > ticksCooldown) {
                        enemyCaught = true;
                        break;
                    }
                }
            }
        }
    }

    return enemyCaught;
}

const Track& determineBestTrack(const vector<Track>& tracks) {
    auto& firstTrack = tracks.front();
    if (!Debug::isFastMode) return firstTrack;

    const Track *firstForward = nullptr;
    for (auto& track : tracks) {
        auto& move = track.moves().front();
        if (move.enginePower == 1.0 && !move.brake) {
            firstForward = &track;
            break;
        }
    }

    // TODO: constant
    return firstForward == nullptr || firstTrack.score() > 1.1 * firstForward->score() ? firstTrack : *firstForward;
}

// TODO: only use them if they are from the previous tick
vector<Track> previousTracksAllTeam[2];

Go solve(const World& world, const Car& self, const vector<Tile>& path) {
    auto startState = State(&world, self.getId());

    auto& previousTracks = previousTracksAllTeam[self.getTeammateIndex()];
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

    auto& bestTrack = determineBestTrack(tracks);
    auto bestMove = bestTrack.moves().empty() ? Go() : bestTrack.moves().front();

#ifdef DEBUG_OUTPUT
    {
        // Debug::debug = true;
        auto state = State(startState);
        for (auto& go : bestTrack.moves()) {
            state.apply(go);
            // Debug::debug = false;
            vis->drawRect(state.me().rectangle);
        }
        vis->drawRect(state.me().rectangle);
        cout << "tick " << world.getTick() << " tracks " << tracks.size() << " best-score " << bestTrack.score() <<
            " best-move " << bestMove.toString() << " " << state.me().toString() << endl;
        scorer.scoreState(state, true);
    }
#endif

    previousTracks.resize(tracks.size());
    transform(tracks.begin(), tracks.end(), previousTracks.begin(), [](const Track& track) {
        return track.rotate();
    });

    if (!(world.getTick() % FIRE_CHECK_PERIOD) && startState.me().projectiles > 0 && startState.me().original->getRemainingProjectileCooldownTicks() == 0) {
        if (shouldFire(startState)) {
            bestMove.throwProjectile = true;
        }
    }

    if (!(world.getTick() % OIL_CHECK_PERIOD) && startState.me().oilCanisters > 0 && startState.me().original->getRemainingOilCooldownTicks() == 0) {
        if (shouldSpillOil(startState)) {
            bestMove.spillOil = true;
        }
    }

    return bestMove;
}

vector<Tile> computePath(const Car& self, const World& world) {
    auto& waypoints = world.getWaypoints();
    int i = self.getNextWaypointIndex();
    DirectedTile start = CarPosition(&world, &self).directedTile();
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

struct SafeModeConfig {
    int lastNonZeroSpeedTick;
    int safeUntilTick;
    int waitUntilTick;
    int lastSafeModeAppliedTick;
    double safeModeEnginePower;
    double lastTickHealth;

    SafeModeConfig() :
        lastNonZeroSpeedTick(Const::getGame().getInitialFreezeDurationTicks()),
        safeUntilTick(-1),
        waitUntilTick(-1),
        lastSafeModeAppliedTick(-1),
        safeModeEnginePower(1.0),
        lastTickHealth(1.0) { }

    static SafeModeConfig& getInstance(const Car *car) {
        static SafeModeConfig instances[] = { SafeModeConfig(), SafeModeConfig() };
        return instances[car->getTeammateIndex()];
    }
};

const double SAFE_MODE_MIN_SPEED = 1.0;
const int SAFE_MODE_TOLERANCE = 10;
const int SAFE_MODE_DURATION = 120;
const int SAFE_MODE_WAIT = 60;

bool pointlessSafeMove(const CarPosition& me, const World& world, Go move, int ticks) {
    auto state = State(&world, me.original->getId());
    for (int i = 0; i < ticks; i++) {
        move.brake = state.me().enginePower * move.enginePower < 0.0;
        state.apply(move);
    }
    // TODO
    /*
    cout << "### before " << me.toString() << endl;
    cout << "### after " << state.me().toString() << endl;
    cout << "### distance " << state.me().location.distanceTo(me.location) << endl;
    */
    return state.me().location.distanceTo(me.location) < Const::getGame().getCarHeight() / 2;
}

bool safeMode(const CarPosition& me, const World& world, Move& move) {
    auto& config = SafeModeConfig::getInstance(me.original);
    int& lastNonZeroSpeedTick = config.lastNonZeroSpeedTick;
    int& safeUntilTick = config.safeUntilTick;
    int& waitUntilTick = config.waitUntilTick;
    int& lastSafeModeAppliedTick = config.lastSafeModeAppliedTick;
    double& safeModeEnginePower = config.safeModeEnginePower;

    auto tick = world.getTick();
    auto nonZeroSpeed = abs(me.velocity.length()) > SAFE_MODE_MIN_SPEED;

    // cout << "tick " << tick << " last-non-zero " << lastNonZeroSpeedTick << " safe-until " << safeUntilTick << " wait-until " << waitUntilTick << " non-zero " << nonZeroSpeed << endl;

    if (tick <= safeUntilTick) {
        lastSafeModeAppliedTick = tick;

        auto path = computePath(*me.original, world);
        auto desiredDirection = Vec(path[0].toPoint(), path[1].toPoint());
        auto go = Go(safeModeEnginePower, me.direction().angleTo(desiredDirection) < 0.0 ? TURN_LEFT : TURN_RIGHT);
        /*
        // TODO: fix and enable
        if (pointlessSafeMove(me, world, go, safeUntilTick - tick)) {
            go.enginePower = 1.0;
        }
        */
        if (go.enginePower > 0.0) {
            go.wheelTurn = go.wheelTurn == TURN_LEFT ? TURN_RIGHT : TURN_LEFT;
        }
        go.brake = me.enginePower * go.enginePower < 0.0;
#ifdef DEBUG_OUTPUT
        cout << "tick " << world.getTick() << " safe-mode " << go.toString() << " " << me.toString() << endl;
#endif
        go.applyTo(*me.original, move);
        return true;
    }

    if (nonZeroSpeed || tick <= safeUntilTick || tick <= waitUntilTick) {
        lastNonZeroSpeedTick = tick;
    }
    
    if (tick <= waitUntilTick || tick - lastNonZeroSpeedTick < SAFE_MODE_TOLERANCE) {
        return false;
    }

    safeUntilTick = tick + SAFE_MODE_DURATION;
    waitUntilTick = safeUntilTick + SAFE_MODE_WAIT;
    safeModeEnginePower = -safeModeEnginePower;

    if (tick - lastSafeModeAppliedTick > SAFE_MODE_WAIT * 3) {
        // If the last safe mode was applied a long time ago, we're in a new accident and should try go backwards first
        safeModeEnginePower = -1.0;
    }

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

    Debug::tick = world.getTick();

    auto& safeModeConfig = SafeModeConfig::getInstance(&self);
    auto& lastTickHealth = safeModeConfig.lastTickHealth;
    if (safeModeConfig.lastTickHealth < 1e-9 && self.getDurability() > safeModeConfig.lastTickHealth) {
        safeModeConfig.safeUntilTick = -1;
        safeModeConfig.lastSafeModeAppliedTick = -1;
        safeModeConfig.waitUntilTick = world.getTick() + SAFE_MODE_WAIT;
    }
    lastTickHealth = self.getDurability();

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

    if (safeMode(CarPosition(&world, &self), world, move)) {
        return;
    }

    auto path = computePath(self, world);

    Go solution = solve(world, self, path);
    solution.applyTo(self, move);

#ifdef VISUALIZE
    printMove(move);
#endif
}

MyStrategy::MyStrategy() { }
