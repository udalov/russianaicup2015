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

double updateWheelTurn(double carWheelTurn, WheelTurnDirection direction) {
    static double maxChange = Const::getGame().getCarWheelTurnChangePerTick();
    switch (direction) {
        case WheelTurnDirection::TURN_LEFT: return max(carWheelTurn - maxChange, -1.0);
        case WheelTurnDirection::KEEP: return carWheelTurn;
        case WheelTurnDirection::TURN_RIGHT: return min(carWheelTurn + maxChange, 1.0);
        default: return carWheelTurn;
    }
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
    static const double invertedBuggyMass = 1.0 / Const::getGame().getBuggyMass();
    static const double invertedJeepMass = 1.0 / Const::getGame().getJeepMass();
    static const double momentumTransferFactor = 0.3; // ?!

    double invertedCarMass = car.isBuggy() ? invertedBuggyMass : invertedJeepMass;

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
    static const double invertedBuggyMass = 1.0 / Const::getGame().getBuggyMass();
    static const double invertedJeepMass = 1.0 / Const::getGame().getJeepMass();
    static const double surfaceFrictionFactor = 0.015; // ?!

    double invertedCarMass = car.isBuggy() ? invertedBuggyMass : invertedJeepMass;

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
    vector<Segment> segments;
    vector<Circle> corners;
    int height;

    Walls(const vector<Segment>& segments, const vector<Circle>& corners) :
        segments(segments), corners(corners), height(Map::getMap().width) { }

    const Segment& segment(unsigned long tx, unsigned long ty, int d) {
        return segments[((tx * height) + ty) * 4 + d];
    }

    const Circle& corner(unsigned long tx, unsigned long ty, int d) {
        return corners[((tx * height) + ty) * 4 + d];
    }

    static Walls *instance;
};

Walls *Walls::instance = nullptr;

Walls *computeWalls() {
    static Game& game = Const::getGame();
    static const double mapWidth = Map::getMap().width;
    static const double mapHeight = Map::getMap().height;
    static const double margin = game.getTrackTileMargin() * 1.01; // To avoid driving right against the wall
    static const double tileSize = game.getTrackTileSize();

    auto segments = vector<Segment>(mapWidth * mapHeight * 4, Segment { Point(), Point() });
    auto circles = vector<Circle>(mapWidth * mapHeight * 4, Circle { Point(), double() });

    static const int dx[] = {1, 0, -1, 0};
    static const int dy[] = {0, 1, 0, -1};

    for (unsigned long tx = 0; tx < mapWidth; tx++) {
        for (unsigned long ty = 0; ty < mapHeight; ty++) {
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

                segments[((tx * mapHeight) + ty) * 4 + d] = segment;

                auto p = Point(
                        (tx + (dx[d] - dy[d] + 1.) / 2) * tileSize,
                        (ty + (dx[d] + dy[d] + 1.) / 2) * tileSize
                );
                circles[((tx * mapHeight) + ty) * 4 + d] = Circle(p, margin);
            }
        }
    }

    Walls::instance = new Walls(segments, circles);

    return Walls::instance;
}

void collideCarWithSegmentWall(CarPosition& car, const Segment& wall, int d) {
    static const double carRadius = myHypot(Const::getGame().getCarHeight() / 2, Const::getGame().getCarWidth() / 2) + EPSILON;

    auto& location = car.location;
    if (!(d & 1)) {
        if (abs(location.x - wall.p1.x) > carRadius) return;
        if (location.y < wall.p1.y - carRadius) return;
        if (location.y > wall.p2.y + carRadius) return;
    } else {
        if (abs(location.y - wall.p1.y) > carRadius) return;
        if (location.x < wall.p1.x - carRadius) return;
        if (location.x > wall.p2.x + carRadius) return;
    }

    CollisionInfo collision;
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

void collideCarWithCornerWall(CarPosition& car, const Circle& corner) {
    static const double carRadius = myHypot(Const::getGame().getCarHeight() / 2, Const::getGame().getCarWidth() / 2) + EPSILON;
    static const double margin = Const::getGame().getTrackTileMargin();

    if (car.location.distanceTo(corner.center) > margin + carRadius) return;
    if (car.rectangle.distanceFrom(corner.center) > margin + EPSILON) return;

    CollisionInfo collision;
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

void determineTileBounds(
        const Rect& rectangle, unsigned long& txBegin, unsigned long& txEnd, unsigned long& tyBegin, unsigned long& tyEnd
) {
    static const double tileSize = Const::getGame().getTrackTileSize();

    auto& p = rectangle.points();

    auto minX = min(min((&p[0])->x, (&p[1])->x), min((&p[2])->x, (&p[3])->x));
    auto maxX = max(max((&p[0])->x, (&p[1])->x), max((&p[2])->x, (&p[3])->x));
    auto minY = min(min((&p[0])->y, (&p[1])->y), min((&p[2])->y, (&p[3])->y));
    auto maxY = max(max((&p[0])->y, (&p[1])->y), max((&p[2])->y, (&p[3])->y));

    txBegin = static_cast<unsigned long>(minX / tileSize);
    txEnd = static_cast<unsigned long>(maxX / tileSize);
    tyBegin = static_cast<unsigned long>(minY / tileSize);
    tyEnd = static_cast<unsigned long>(maxY / tileSize);
}

void collideCarWithWalls(CarPosition& car) {
    static Map& map = Map::getMap();

    static Walls *allWalls = computeWalls();

    unsigned long txBegin, txEnd, tyBegin, tyEnd;
    determineTileBounds(car.rectangle, txBegin, txEnd, tyBegin, tyEnd);

    for (auto tx = txBegin; tx <= txEnd; tx++) {
        for (auto ty = tyBegin; ty <= tyEnd; ty++) {
            int tile = map.get(tx, ty);

            for (int d = 0; d < 4; d++) {
                if (!(tile & (1 << d))) {
                    collideCarWithSegmentWall(car, allWalls->segment(tx, ty, d), d);
                }
                if ((tile & (1 << d)) && (tile & (1 << ((d + 1) & 3)))) {
                    collideCarWithCornerWall(car, allWalls->corner(tx, ty, d));
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

    static const double jeepEngineRearAcceleration = game.getJeepEngineRearPower() / game.getJeepMass();
    static const double jeepEngineForwardAcceleration = game.getJeepEngineForwardPower() / game.getJeepMass();

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
    for (auto& point : rectangle.points()) {
        point += velocity;
    }

    if (!move.brake) {
        auto acceleration = enginePower * (enginePower < 0 ?
                (isBuggy() ? buggyEngineRearAcceleration : jeepEngineRearAcceleration):
                (isBuggy() ? buggyEngineForwardAcceleration : jeepEngineForwardAcceleration)
        );
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

    angle = normalizeAngle(angle + angularSpeed);
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

bool CarPosition::isBuggy() const {
    return original->getType() == model::CarType::BUGGY;
}

DirectedTile CarPosition::directedTile() const {
    double positiveAngle = normalizeAngle(angle);
    if (positiveAngle < 0) positiveAngle += M_PI + M_PI;
    int direction = static_cast<int>(positiveAngle * 4.0 / M_PI + 0.5) % 8;
    return DirectedTile(tile(), direction);
}

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
