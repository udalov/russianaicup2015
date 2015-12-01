#include "State.h"
#include "Collider.h"
#include "Const.h"
#include "Debug.h"
#include "Map.h"
#include "math3d.h"

#include <algorithm>
#include <memory>
#include <sstream>
#include <iostream>

using namespace std;

const double EPSILON = 1e-7;

State::State(const World *world) : original(world) {
    auto& cars = world->getCars();
    this->cars.reserve(cars.size());
    for (bool teammates : { true, false }) {
        for (unsigned long i = 0, size = cars.size(); i < size; i++) {
            if (teammates == (world->getMyPlayer().getId() == cars[i].getPlayerId())) {
                this->cars.emplace_back(&cars[i]);
            }
        }
    }

    auto& oilSlicks = world->getOilSlicks();
    this->oilSlicks.reserve(oilSlicks.size());
    for (unsigned long i = 0, size = oilSlicks.size(); i < size; i++) {
        this->oilSlicks.emplace_back(&oilSlicks[i]);
    }

    auto& washers = world->getProjectiles();
    for (unsigned long i = 0, size = washers.size(); i < size; i++) {
        this->washers.emplace_back(&washers[i]);
    }
}

double updateEnginePower(double carEnginePower, double moveEnginePower) {
    static double maxChange = Const::getGame().getCarEnginePowerChangePerTick();
    auto delta = moveEnginePower - carEnginePower;
    return carEnginePower + min(max(delta, -maxChange), maxChange);
}

double updateWheelTurn(double carWheelTurn, double moveWheelTurn) {
    static double maxChange = Const::getGame().getCarWheelTurnChangePerTick();
    auto delta = moveWheelTurn - carWheelTurn;
    return carWheelTurn + min(max(delta, -maxChange), maxChange);
}

Vec3D toVec3D(const Vec& vec) {
    return Vec3D(vec.x, vec.y, 0.0);
}

Vec3D toVec3D(const Point& p1, const Point& p2) {
    return toVec3D(Vec(p1, p2));
}

void resolveImpact(
        const CollisionInfo& collision, CarPosition& car, const Vec3D& normal, const Vec3D& vecBC, const Vec3D& relativeVelocity
) {
    static const double invertedCarMass = 1.0 / Const::getGame().getBuggyMass();
    static const double momentumTransferFactor = 0.3; // ?!

    double invertedCarAngularMass = car.angularSpeed * invertedCarMass / car.velocity.length();

    auto part = ((vecBC ^ normal) * invertedCarAngularMass) ^ vecBC;
    auto denominator = invertedCarMass + normal * part;

    auto impulseChange = -1.0 * (1.0 + momentumTransferFactor) * (relativeVelocity * normal) / denominator;
    // if (D) cout << "impact impulse-change " << impulseChange << endl;
    if (abs(impulseChange) < EPSILON) return;

    auto velocityChange = normal * (impulseChange * invertedCarMass);
    auto newVelocity = toVec3D(car.velocity) - velocityChange;
    // if (D) cout << "impact previous-velocity " << car.velocity.toString() << endl;
    car.velocity = Vec(newVelocity.x, newVelocity.y);

    // if (D) cout << "impact velocity-change " << velocityChange.x << " " << velocityChange.y << " new-velocity " << car.velocity.toString() << endl;

    auto angularSpeedChange = (vecBC ^ (normal * impulseChange)) * invertedCarAngularMass;
    car.angularSpeed -= angularSpeedChange.z;
    // if (D) cout << "impact angular-speed-change " << angularSpeedChange.z << " new-angular-speed " << car.angularSpeed << endl;
}

void resolveSurfaceFriction(
        const CollisionInfo& collision, CarPosition& car, const Vec3D& normal, const Vec3D& vecBC, const Vec3D& relativeVelocity
) {
    static const double invertedCarMass = 1.0 / Const::getGame().getBuggyMass();
    static const double surfaceFrictionFactor = 0.015; // ?!

    double invertedCarAngularMass = car.angularSpeed * invertedCarMass / car.velocity.length();

    auto tangent = relativeVelocity - normal * (relativeVelocity * normal);
    if (tangent.normSquare() < EPSILON * EPSILON) return;

    tangent = tangent.normalize();
    auto surfaceFriction = mySqrt(2 * surfaceFrictionFactor) * abs(relativeVelocity * normal) / relativeVelocity.norm();
    if (surfaceFriction < EPSILON) return;

    auto part = ((vecBC ^ tangent) * invertedCarAngularMass) ^ vecBC;
    auto denominator = invertedCarMass + tangent * part;

    auto impulseChange = -1.0 * surfaceFriction * (relativeVelocity * tangent) / denominator;
    // if (D) cout << "surface-friction impulse-change " << impulseChange << endl;
    if (abs(impulseChange) < EPSILON) return;

    auto velocityChange = tangent * (impulseChange * invertedCarMass);
    auto newVelocity = toVec3D(car.velocity) - velocityChange;
    // if (D) cout << "surface-friction previous-velocity " << car.velocity.toString() << endl;
    car.velocity = Vec(newVelocity.x, newVelocity.y);

    // if (D) cout << "surface-friction velocity-change " << velocityChange.x << " " << velocityChange.y << " new-velocity " << car.velocity.toString() << endl;

    auto angularSpeedChange = (vecBC ^ (tangent * impulseChange)) * invertedCarAngularMass;
    car.angularSpeed -= angularSpeedChange.z;
    // if (D) cout << "surface-friction angular-speed-change " << angularSpeedChange.z << " new-angular-speed " << car.angularSpeed << endl;
}

void resolveWallCollision(const CollisionInfo& collision, CarPosition& car) {
    // if (D) cout << "  velocity " << car.velocity.length() << endl;
    car.health = max(car.health - car.velocity.length() / 200.0, 0.0); // ?!

    // TODO: turn on
    /*
    auto normal = toVec3D(collision.normal);
    auto vecBC = toVec3D(car.location, collision.point);
    auto angularSpeedBC = Vec3D(0.0, 0.0, car.angularSpeed) ^ vecBC;
    auto velocityBC = toVec3D(car.velocity) + angularSpeedBC;
    auto relativeVelocity = -velocityBC;
    // if (D) cout << "vec-bc " << vecBC.toString() << " relative-velocity " << relativeVelocity.toString() << endl;
    if (relativeVelocity * normal < EPSILON) {
        // TODO: optimize
        resolveImpact(collision, car, normal, vecBC, relativeVelocity);
        resolveSurfaceFriction(collision, car, normal, vecBC, relativeVelocity);
    }
    */

    // TODO: dirty hack
    auto speed = car.velocity.length();
    if (speed > EPSILON) {
        car.velocity *= (car.velocity * collision.normal) / speed;
    }

    if (collision.depth > EPSILON) {
        car.location -= collision.normal * (collision.depth + EPSILON);
    }
}

struct Walls {
    vector<vector<vector<Segment>>> segments;
    vector<vector<vector<Circle>>> corners;

    Walls(const vector<vector<vector<Segment>>>& segments, const vector<vector<vector<Circle>>>& corners) :
        segments(segments), corners(corners) { }

    static Walls *instance;
};

Walls *Walls::instance = nullptr;

Walls *computeWalls() {
    static Game& game = Const::getGame();
    static Map& map = Map::getMap();
    static const double margin = game.getTrackTileMargin() * 1.01; // To avoid driving right against the wall
    static const double tileSize = game.getTrackTileSize();

    auto segments = vector<vector<vector<Segment>>>(map.width, {
            vector<vector<Segment>>(map.height, {
                vector<Segment>(4, Segment { Point(), Point() })
            })
    });
    auto circles = vector<vector<vector<Circle>>>(map.width, {
            vector<vector<Circle>>(map.height, {
                vector<Circle>(4, Circle { Point(), double() })
            })
    });

    static const int dx[] = {1, 0, -1, 0};
    static const int dy[] = {0, 1, 0, -1};

    for (unsigned long tx = 0; tx < map.width; tx++) {
        for (unsigned long ty = 0; ty < map.height; ty++) {
            for (int d = 0; d < 4; d++) {
                auto p1 = Point(
                        (tx + max(dx[d] + dy[d], 0)) * tileSize - dx[d] * margin,
                        (ty + max(dy[d] - dx[d], 0)) * tileSize - dy[d] * margin
                );
                auto p2 = Point(
                        (tx + max(dx[d] - dy[d], 0)) * tileSize - dx[d] * margin,
                        (ty + max(dx[d] + dy[d], 0)) * tileSize - dy[d] * margin
                );
                auto segment = Segment(p1, p2);

                // Reorder p1 and p2 so that if p1.x == p2.x then p1.y < p2.y, and if p1.y == p2.y then p1.x < p2.x
                if (!(d & 1) && segment.p1.y > segment.p2.y) segment = Segment(segment.p2, segment.p1);
                if ((d & 1) && segment.p1.x > segment.p2.x) segment = Segment(segment.p2, segment.p1);

                segments[tx][ty][d] = segment;

                auto p = Point(
                        (tx + (dx[d] - dy[d] + 1.) / 2) * tileSize,
                        (ty + (dx[d] + dy[d] + 1.) / 2) * tileSize
                );
                circles[tx][ty][d] = Circle(p, margin);
            }
        }
    }

    Walls::instance = new Walls(segments, circles);

    return Walls::instance;
}

void collideCarWithWalls(CarPosition& car) {
    static Game& game = Const::getGame();
    static Map& map = Map::getMap();
    static const double margin = game.getTrackTileMargin();
    static const double tileSize = game.getTrackTileSize();
    static const double carRadius = myHypot(game.getCarHeight() / 2, game.getCarWidth() / 2) + EPSILON;

    static auto allWalls = computeWalls();

    auto& location = car.location;

    unsigned long txBegin = map.width - 1, txEnd = 0, tyBegin = map.height - 1, tyEnd = 0;
    for (auto& point : car.rectangle.points) {
        auto px = static_cast<unsigned long>(point.x / tileSize);
        auto py = static_cast<unsigned long>(point.y / tileSize);
        txBegin = min(txBegin, px);
        txEnd = max(txEnd, px);
        tyBegin = min(tyBegin, py);
        tyEnd = max(tyEnd, py);
    }
    txBegin = max(txBegin, 0ul);
    txEnd = min(txEnd, map.width - 1);
    tyBegin = max(tyBegin, 0ul);
    tyEnd = min(tyEnd, map.height - 1);

    CollisionInfo collision;
    for (auto tx = txBegin; tx <= txEnd; tx++) {
        for (auto ty = tyBegin; ty <= tyEnd; ty++) {
            auto tile = map.graph[tx][ty];

            auto& wallSegments = allWalls->segments[tx][ty];
            for (int d = 0; d < 4; d++) {
                if (tile & (1 << d)) continue;

                auto& wall = wallSegments[d];
                if (!(d & 1)) {
                    if (abs(location.x - wall.p1.x) > carRadius) continue;
                    if (location.y < wall.p1.y - carRadius) continue;
                    if (location.y > wall.p2.y + carRadius) continue;
                } else {
                    if (abs(location.y - wall.p1.y) > carRadius) continue;
                    if (location.x < wall.p1.x - carRadius) continue;
                    if (location.x > wall.p2.x + carRadius) continue;
                }

                if (collideRectAndSegment(car.rectangle, wall, collision)) {
                    /*
                    if (D) {
                        cout << "segment collision at " << collision.point.toString() << " normal " << collision.normal.toString() << " depth " << collision.depth << endl;
                        cout << "  " << car.toString() << endl;
                        cout << "  (with segment " << wall.p1.toString() << " -> " << wall.p2.toString() << ")" << endl;
                    }
                    */
                    resolveWallCollision(collision, car);
                }
            }

            auto& wallCorners = allWalls->corners[tx][ty];
            for (int d = 0; d < 4; d++) {
                if (!(tile & (1 << d)) || !(tile & (1 << ((d + 1) & 3)))) continue;
                auto& corner = wallCorners[d];
                if (location.distanceTo(corner.center) > margin + carRadius) continue;
                if (car.rectangle.distanceFrom(corner.center) > margin + EPSILON) continue;
                if (collideCircleAndRect(car.rectangle, corner, collision)) {
                    /*
                    if (D) {
                        cout << "corner collision at " << collision.point.toString() << " normal " << collision.normal.toString() << " depth " << collision.depth << endl;
                        cout << "  " << car.toString() << endl;
                        cout << "  (with corner at " << p.toString() << ")" << endl;
                    }
                    */
                    resolveWallCollision(collision, car);
                }
            }
        }
    }
}

void State::apply(const Go& move) {
    // TODO: simulate all cars
    cars.front().advance(move);
    collideCarWithWalls(cars.front());

    for (auto& oilSlick : oilSlicks) {
        oilSlick.apply();
    }

    for (auto& washer : washers) {
        // TODO: add the condition when washers disappear
        washer.apply();
    }
}

const CarPosition& State::me() const {
    return cars.front();
}

bool OilSlickPosition::isAlive() const {
    return remainingTime > 0 /* or 1? */;
}

void OilSlickPosition::apply() {
    remainingTime--;
}

void WasherPosition::apply() {
    // TODO
}

Rect rectangleByCar(double angle, double x, double y) {
    static const double carWidth = Const::getGame().getCarWidth();
    static const double carHeight = Const::getGame().getCarHeight();

    Vec dir = Vec(angle);
    Vec forward = dir * (carWidth / 2);
    Vec sideways = Vec(-dir.y, dir.x) * (carHeight / 2);
    Point lpf = Point(x + forward.x, y + forward.y);
    Point lmf = Point(x - forward.x, y - forward.y);
    return Rect({ lpf - sideways, lpf + sideways, lmf + sideways, lmf - sideways });
}

void CarPosition::advance(const Go& move) {
    static auto& game = Const::getGame();

    static const double carAngularSpeedFactor = game.getCarAngularSpeedFactor();
    static const int nitroDurationTicks = game.getNitroDurationTicks();
    static const double nitroEnginePowerFactor = game.getNitroEnginePowerFactor();

    static const double buggyEngineRearAcceleration = game.getBuggyEngineRearPower() / game.getBuggyMass();
    static const double buggyEngineForwardAcceleration = game.getBuggyEngineForwardPower() / game.getBuggyMass();

    static const double airFriction = 1.0 - game.getCarMovementAirFrictionFactor();
    static const double lengthwiseVelocityChangeBase = game.getCarLengthwiseMovementFrictionFactor();
    static const double crosswiseVelocityChange = game.getCarCrosswiseMovementFrictionFactor();

    auto dir = direction();
    auto side = Vec(-dir.y, dir.x);

    // Engine & wheels

    if (move.useNitro && nitroCharges > 0 && nitroCooldown == 0) {
        nitroCharges--;
        nitroCooldown = nitroDurationTicks;
    }

    if (nitroCooldown > 0) {
        nitroCooldown--;
        enginePower = nitroEnginePowerFactor;
    } else {
        enginePower = updateEnginePower(enginePower == nitroEnginePowerFactor ? 1.0 : enginePower, move.enginePower);
    }

    wheelTurn = updateWheelTurn(wheelTurn, move.wheelTurn);
    angularSpeed = wheelTurn * carAngularSpeedFactor * (velocity * dir);

    // Location

    location += velocity;
    for (auto& point : rectangle.points) {
        point += velocity;
    }

    if (!move.brake) {
        auto acceleration = enginePower * (enginePower < 0 ? buggyEngineRearAcceleration : buggyEngineForwardAcceleration);
        velocity += dir * acceleration;
    }

    // Air friction

    velocity *= airFriction;

    // Movement friction

    auto lengthwiseVelocityChange = move.brake ? crosswiseVelocityChange : lengthwiseVelocityChangeBase;
    auto lengthwiseVelocityPart = velocity * dir;
    lengthwiseVelocityPart =
            lengthwiseVelocityPart >= 0.0
            ? max(lengthwiseVelocityPart - lengthwiseVelocityChange, 0.0)
            : min(lengthwiseVelocityPart + lengthwiseVelocityChange, 0.0);

    auto crosswiseVelocityPart = velocity * side;
    crosswiseVelocityPart =
            crosswiseVelocityPart >= 0.0
            ? max(crosswiseVelocityPart - crosswiseVelocityChange, 0.0)
            : min(crosswiseVelocityPart + crosswiseVelocityChange, 0.0);

    velocity = dir * lengthwiseVelocityPart + side * crosswiseVelocityPart;

    // Angle

    angle += angularSpeed;
}

CarPosition::CarPosition(const Car *car) : original(car),
        location(Point(car->getX(), car->getY())),
        velocity(Vec(car->getSpeedX(), car->getSpeedY())),
        angle(car->getAngle()),
        angularSpeed(car->getAngularSpeed()),
        enginePower(car->getEnginePower()),
        wheelTurn(car->getWheelTurn()),
        health(car->getDurability()),
        nitroCharges(car->getNitroChargeCount()),
        nitroCooldown(car->getRemainingNitroCooldownTicks()),
        rectangle(rectangleByCar(car->getAngle(), car->getX(), car->getY())) { }

Point CarPosition::bumperCenter() const {
    static const double carWidth = Const::getGame().getCarWidth();

    return location + direction() * (carWidth / 2);
}

Tile CarPosition::tile() const {
    static const double tileSize = Const::getGame().getTrackTileSize();
    static const int width = Map::getMap().width;
    static const int height = Map::getMap().height;

    Point bc = bumperCenter();
    return Tile(
            max(min(static_cast<int>(bc.x / tileSize), width - 1), 0),
            max(min(static_cast<int>(bc.y / tileSize), height - 1), 0)
    );
};

string CarPosition::toString() const {
    ostringstream ss;
    ss.precision(3);
    ss << fixed << "car at " << location.toString() <<
            " going " << velocity.toString() <<
            " angle " << angle <<
            " angular " << angularSpeed <<
            " engine " << enginePower <<
            " wheel " << wheelTurn <<
            " health " << health <<
            " nitros " << nitroCharges <<
            " nitro-cooldown " << nitroCooldown;
    return ss.str();
}
