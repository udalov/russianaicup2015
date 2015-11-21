#include "State.h"
#include "Collider.h"
#include "Const.h"
#include "Map.h"
#include "math3d.h"

#include <algorithm>
#include <sstream>
#include <iostream>

using namespace std;

const int ITERATION_COUNT_PER_STEP = 2;
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
    static Game& game = Const::getGame();
    static const double invertedCarMass = 1.0 / game.getBuggyMass();
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
    static Game& game = Const::getGame();
    static const double invertedCarMass = 1.0 / game.getBuggyMass();
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
    if (relativeVelocity * normal < EPSILON) {
        // TODO: optimize
        resolveImpact(collision, car, normal, vecBC, relativeVelocity);
        resolveSurfaceFriction(collision, car, normal, vecBC, relativeVelocity);
    }
    if (collision.depth > EPSILON) {
        car.location -= collision.normal * (collision.depth + EPSILON);
    }
}

void collideCarWithWalls(CarPosition& car) {
    static Game& game = Const::getGame();
    static Map& map = Map::getMap();
    static const double margin = game.getTrackTileMargin();
    static const double tileSize = game.getTrackTileSize();
    static const double carRadius = myHypot(game.getCarHeight() / 2, game.getCarWidth() / 2);

    static const int dx[] = {1, 0, -1, 0};
    static const int dy[] = {0, 1, 0, -1};

    auto rect = car.rectangle();

    auto tileX = static_cast<unsigned long>(car.location.x / tileSize - 0.5);
    auto tileY = static_cast<unsigned long>(car.location.y / tileSize - 0.5);
    for (auto tx = tileX; tx <= min(tileX + 1, map.width - 1); tx++) {
        for (auto ty = tileY; ty <= min(tileY + 1, map.height - 1); ty++) {
            auto tile = map.graph[tx][ty];
            for (int d = 0; d < 4; d++) {
                if (tile & (1 << d)) continue;
                auto p1 = Point(
                        (tx + max(dx[d] + dy[d], 0)) * tileSize - dx[d] * margin,
                        (ty + max(dy[d] - dx[d], 0)) * tileSize - dy[d] * margin
                );
                auto p2 = Point(
                        (tx + max(dx[d] - dy[d], 0)) * tileSize - dx[d] * margin,
                        (ty + max(dx[d] + dy[d], 0)) * tileSize - dy[d] * margin
                );
                auto wall = Segment(p1, p2);
                if (wall.distanceFrom(car.location) > carRadius + EPSILON) continue;
                CollisionInfo collision;
                if (collideRectAndSegment(rect, wall, collision)) {
                    /*
                    if (D) {
                        cout << "segment collision at " << collision.point.toString() << " normal " << collision.normal.toString() << " depth " << collision.depth << endl;
                        cout << "  (with segment " << wall.p1.toString() << " -> " << wall.p2.toString() << ")" << endl;
                    }
                    */
                    resolveWallCollision(collision, car);
                }
            }

            for (int d = 0; d < 4; d++) {
                if (!(tile & (1 << d)) || !(tile & (1 << ((d + 1) & 3)))) continue;
                auto p = Point(
                        (tx + (dx[d] - dy[d] + 1.) / 2) * tileSize,
                        (ty + (dx[d] + dy[d] + 1.) / 2) * tileSize
                );
                if (rect.distanceFrom(p) > margin + EPSILON) continue;
                auto corner = Circle(p, margin);
                CollisionInfo collision;
                if (collideCircleAndRect(rect, corner, collision)) {
                    /*
                    if (D) {
                        cout << "corner collision at " << collision.point.toString() << " normal " << collision.normal.toString() << " depth " << collision.depth << endl;
                        cout << "  (me at " << car.location.toString() << " with corner at " << p.toString() << ")" << endl;
                    }
                    */
                    resolveWallCollision(collision, car);
                }
            }
        }
    }
}

void State::apply(const vector<Go>& moves) {
    const double updateFactor = 1.0 / ITERATION_COUNT_PER_STEP;
    // TODO: simulate all cars
    const bool simulateAllCars = false;

    vector<double> medianAngularSpeed(cars.size());
    for (unsigned long i = 0, size = simulateAllCars ? cars.size() : 1; i < size; i++) {
        auto& car = cars[i];
        auto& move = moves[i];

        if (move.useNitro && car.nitroCharges > 0 && car.nitroCooldown == 0) {
            car.nitroCharges--;
            car.nitroCooldown = Const::getGame().getNitroDurationTicks();
        }

        if (car.nitroCooldown > 0) {
            car.nitroCooldown--;
            car.enginePower = Const::getGame().getNitroEnginePowerFactor();
        } else {
            car.enginePower = updateEnginePower(
                    car.enginePower == Const::getGame().getNitroEnginePowerFactor() ? 1.0 : car.enginePower,
                    move.enginePower
            );
        }

        car.wheelTurn = updateWheelTurn(car.wheelTurn, move.wheelTurn);
        car.angularSpeed =
                car.wheelTurn * Const::getGame().getCarAngularSpeedFactor() *
                (car.velocity * car.direction());

        medianAngularSpeed[i] = car.angularSpeed;
    }

    for (int iteration = 1; iteration <= ITERATION_COUNT_PER_STEP; iteration++) {
        for (unsigned long i = 0, size = simulateAllCars ? cars.size() : 1; i < size; i++) {
            cars[i].advance(moves[i], medianAngularSpeed[i], updateFactor);
        }

        for (unsigned long i = 0, size = simulateAllCars ? cars.size() : 1; i < size; i++) {
            collideCarWithWalls(cars[i]);
        }
    }

    for (auto& oilSlick : oilSlicks) {
        oilSlick.apply();
    }

    // TODO: washers should also move for several iterations per step
    for (auto& washer : washers) {
        // TODO: add the condition when washers disappear
        washer.apply();
    }
}

const CarPosition& State::myCar() const {
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

void CarPosition::advance(const Go& move, double medianAngularSpeed, double updateFactor) {
    auto game = Const::getGame();

    // Location

    location += velocity * updateFactor;

    if (!move.brake) {
        auto acceleration = enginePower *
                            (enginePower < 0 ? game.getBuggyEngineRearPower() : game.getBuggyEngineForwardPower()) /
                            game.getBuggyMass();
        velocity += direction() * acceleration * updateFactor;
    }

    // Air friction

    const double airFriction = pow(1.0 - game.getCarMovementAirFrictionFactor(), updateFactor);
    velocity *= airFriction;

    // Movement friction

    const double lengthwiseVelocityChange =
            (move.brake ? game.getCarCrosswiseMovementFrictionFactor() : game.getCarLengthwiseMovementFrictionFactor())
            * updateFactor;
    auto lengthwiseUnitVector = direction();
    auto lengthwiseVelocityPart = velocity * lengthwiseUnitVector;
    lengthwiseVelocityPart =
            lengthwiseVelocityPart >= 0.0
            ? max(lengthwiseVelocityPart - lengthwiseVelocityChange, 0.0)
            : min(lengthwiseVelocityPart + lengthwiseVelocityChange, 0.0);

    const double crosswiseVelocityChange = game.getCarCrosswiseMovementFrictionFactor() * updateFactor;
    auto crosswiseUnitVector = Vec(angle + M_PI / 2);
    auto crosswiseVelocityPart = velocity * crosswiseUnitVector;
    crosswiseVelocityPart =
            crosswiseVelocityPart >= 0.0
            ? max(crosswiseVelocityPart - crosswiseVelocityChange, 0.0)
            : min(crosswiseVelocityPart + crosswiseVelocityChange, 0.0);

    velocity = lengthwiseUnitVector * lengthwiseVelocityPart + crosswiseUnitVector * crosswiseVelocityPart;

    // Angle

    angle += angularSpeed * updateFactor;

    // Rotation air friction

    angularSpeed -= medianAngularSpeed;

    const double rotationAirFriction = pow(1.0 - game.getCarRotationAirFrictionFactor(), updateFactor);
    angularSpeed *= rotationAirFriction;

    if (abs(angularSpeed) < EPSILON) angularSpeed = 0.0;

    // Rotation friction

    const double rotationFrictionFactor = game.getCarRotationFrictionFactor() * updateFactor;
    angularSpeed =
            angularSpeed > 0.0
            ? max(angularSpeed - rotationFrictionFactor, 0.0)
            : min(angularSpeed + rotationFrictionFactor, 0.0);

    angularSpeed += medianAngularSpeed;
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
