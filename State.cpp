#include "State.h"
#include "Collider.h"
#include "Const.h"
#include "Map.h"
#include "math3d.h"

#include <algorithm>
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
    if (collision.depth > EPSILON) {
        car.location -= collision.normal * (collision.depth + EPSILON);
    }
}

struct Walls {
    vector<vector<vector<Segment>>> segments;
    vector<vector<vector<Circle>>> corners;

    Walls(const vector<vector<vector<Segment>>>& segments, const vector<vector<vector<Circle>>>& corners) :
        segments(segments), corners(corners) { }
};

Walls computeWalls() {
    static Game& game = Const::getGame();
    static Map& map = Map::getMap();
    static const double margin = game.getTrackTileMargin();
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
                segments[tx][ty][d] = Segment(p1, p2);

                auto p = Point(
                        (tx + (dx[d] - dy[d] + 1.) / 2) * tileSize,
                        (ty + (dx[d] + dy[d] + 1.) / 2) * tileSize
                );
                circles[tx][ty][d] = Circle(p, margin);
            }
        }
    }

    return Walls(segments, circles);
}

void collideCarWithWalls(CarPosition& car) {
    static Game& game = Const::getGame();
    static Map& map = Map::getMap();
    static const double margin = game.getTrackTileMargin();
    static const double tileSize = game.getTrackTileSize();
    static const double carRadius = myHypot(game.getCarHeight() / 2, game.getCarWidth() / 2);

    static auto allWalls = computeWalls();

    auto tileX = static_cast<unsigned long>(car.location.x / tileSize - 0.5);
    auto tileY = static_cast<unsigned long>(car.location.y / tileSize - 0.5);
    CollisionInfo collision;
    for (auto tx = tileX; tx <= tileX + 1 && tx < map.width; tx++) {
        for (auto ty = tileY; ty <= tileY + 1 && ty < map.height; ty++) {
            auto tile = map.graph[tx][ty];

            auto& wallSegments = allWalls.segments[tx][ty];
            for (int d = 0; d < 4; d++) {
                if (tile & (1 << d)) continue;
                auto& wall = wallSegments[d];
                if (wall.distanceFrom(car.location) > carRadius + EPSILON) continue;
                if (collideRectAndSegment(car.rectangle(), wall, collision)) {
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

            auto& wallCorners = allWalls.corners[tx][ty];
            for (int d = 0; d < 4; d++) {
                if (!(tile & (1 << d)) || !(tile & (1 << ((d + 1) & 3)))) continue;
                auto& corner = wallCorners[d];
                if (car.location.distanceTo(corner.center) > margin + carRadius + EPSILON) continue;
                auto rect = car.rectangle();
                if (rect.distanceFrom(corner.center) > margin + EPSILON) continue;
                if (collideCircleAndRect(rect, corner, collision)) {
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

void State::apply(const vector<Go>& moves) {
    // TODO: simulate all cars
    static const bool carsSize = 1;

    static auto& game = Const::getGame();
    static const double carAngularSpeedFactor = game.getCarAngularSpeedFactor();
    static const int nitroDurationTicks = game.getNitroDurationTicks();
    static const double nitroEnginePowerFactor = game.getNitroEnginePowerFactor();

    for (unsigned long i = 0; i < carsSize; i++) {
        auto& car = cars[i];
        auto& move = moves[i];

        if (move.useNitro && car.nitroCharges > 0 && car.nitroCooldown == 0) {
            car.nitroCharges--;
            car.nitroCooldown = nitroDurationTicks;
        }

        if (car.nitroCooldown > 0) {
            car.nitroCooldown--;
            car.enginePower = nitroEnginePowerFactor;
        } else {
            car.enginePower = updateEnginePower(car.enginePower == nitroEnginePowerFactor ? 1.0 : car.enginePower, move.enginePower);
        }

        car.wheelTurn = updateWheelTurn(car.wheelTurn, move.wheelTurn);
        car.angularSpeed = car.wheelTurn * carAngularSpeedFactor * (car.velocity * car.direction());
    }

    for (unsigned long i = 0; i < carsSize; i++) {
        cars[i].advance(moves[i]);
    }

    for (unsigned long i = 0; i < carsSize; i++) {
        collideCarWithWalls(cars[i]);
    }

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

void CarPosition::advance(const Go& move) {
    static auto& game = Const::getGame();
    static const double buggyEngineRearAcceleration = game.getBuggyEngineRearPower() / game.getBuggyMass();
    static const double buggyEngineForwardAcceleration = game.getBuggyEngineForwardPower() / game.getBuggyMass();

    static const double airFriction = 1.0 - game.getCarMovementAirFrictionFactor();
    static const double lengthwiseVelocityChangeBase = game.getCarLengthwiseMovementFrictionFactor();
    static const double crosswiseVelocityChange = game.getCarCrosswiseMovementFrictionFactor();

    // Location

    location += velocity;

    if (!move.brake) {
        auto acceleration = enginePower * (enginePower < 0 ? buggyEngineRearAcceleration : buggyEngineForwardAcceleration);
        velocity += direction() * acceleration;
    }

    // Air friction

    velocity *= airFriction;

    // Movement friction

    auto lengthwiseVelocityChange = move.brake ? crosswiseVelocityChange : lengthwiseVelocityChangeBase;
    auto lengthwiseUnitVector = direction();
    auto lengthwiseVelocityPart = velocity * lengthwiseUnitVector;
    lengthwiseVelocityPart =
            lengthwiseVelocityPart >= 0.0
            ? max(lengthwiseVelocityPart - lengthwiseVelocityChange, 0.0)
            : min(lengthwiseVelocityPart + lengthwiseVelocityChange, 0.0);

    auto crosswiseUnitVector = Vec(-lengthwiseUnitVector.y, lengthwiseUnitVector.x);
    auto crosswiseVelocityPart = velocity * crosswiseUnitVector;
    crosswiseVelocityPart =
            crosswiseVelocityPart >= 0.0
            ? max(crosswiseVelocityPart - crosswiseVelocityChange, 0.0)
            : min(crosswiseVelocityPart + crosswiseVelocityChange, 0.0);

    velocity = lengthwiseUnitVector * lengthwiseVelocityPart + crosswiseUnitVector * crosswiseVelocityPart;

    // Angle

    angle += angularSpeed;
}

void State::apply(const Go& move) {
    apply(vector<Go>(original->getCars().size(), move));
}

Rect CarPosition::rectangle() const {
    Vec forward = direction() * (original->getWidth() / 2);
    Vec sideways = direction().rotate(M_PI / 2) * (original->getHeight() / 2);
    return Rect({
        location + forward - sideways,
        location + forward + sideways,
        location - forward + sideways,
        location - forward - sideways
    });
}

string CarPosition::toString() const {
    ostringstream ss;
    ss.precision(8);
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
