#include "State.h"
#include "Const.h"
#include "Map.h"

#include <algorithm>
#include <sstream>
#include <iostream>

using namespace std;

const int ITERATION_COUNT_PER_STEP = 10;
const double EPSILON = 1e-7;

State::State(const World *world) : original(world) {
    auto& cars = world->getCars();
    this->cars.reserve(cars.size());
    for (bool teammates : { true, false }) {
        for (unsigned long i = 0, size = cars.size(); i < size; i++) {
            if (teammates == (world->getMyPlayer().getId() == cars[i].getPlayerId())) {
                this->cars.push_back(CarPosition(&cars[i]));
            }
        }
    }

    auto& oilSlicks = world->getOilSlicks();
    this->oilSlicks.reserve(oilSlicks.size());
    for (unsigned long i = 0, size = oilSlicks.size(); i < size; i++) {
        this->oilSlicks.push_back(OilSlickPosition(&oilSlicks[i]));
    }

    auto& washers = world->getProjectiles();
    for (unsigned long i = 0, size = washers.size(); i < size; i++) {
        this->washers.push_back(WasherPosition(&washers[i]));
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

bool hitsTheWall(const CarPosition& car) {
    auto& game = Const::getGame();
    auto& map = Map::getMap();
    const double margin = game.getTrackTileMargin();
    const double tileSize = game.getTrackTileSize();

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
                if (Segment(p1, p2).intersects(rect)) {
                    return true;
                }
            }

            for (int d = 0; d < 4; d++) {
                if ((tile & (1 << d)) && (tile & (1 << ((d + 1) & 3)))) {
                    auto p = Point(
                            (tx + (dx[d] - dy[d] + 1.) / 2) * tileSize,
                            (ty + (dx[d] + dy[d] + 1.) / 2) * tileSize
                    );
                    if (rect.distanceFrom(p) < margin + EPSILON) {
                        return true;
                    }
                }
            }
        }
    }

    return false;
}

State State::apply(const vector<Go>& moves) const {
    const double updateFactor = 1.0 / ITERATION_COUNT_PER_STEP;
    // TODO: simulate all cars
    const bool simulateAllCars = false;

    vector<CarPosition> cars(this->cars);
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
    }

    // TODO: this is temporary
    if (hitsTheWall(cars.front())) {
        cars.front().health = 0.0;
    }

    vector<OilSlickPosition> oilSlicks;
    for (auto& oilSlick : this->oilSlicks) {
        if (oilSlick.remainingTime > 0 /* or 1? */) {
            oilSlicks.push_back(oilSlick.apply());
        }
    }

    // TODO: washers should also move for several iterations per step
    vector<WasherPosition> washers;
    for (auto& washer : this->washers) {
        // TODO: add the condition when washers disappear
        washers.push_back(washer.apply());
    }

    return State(original, cars, oilSlicks, washers);
}

OilSlickPosition OilSlickPosition::apply() const {
    return OilSlickPosition(this->original, this->remainingTime - 1);
}

WasherPosition WasherPosition::apply() const {
    // TODO
    return WasherPosition(this->original);
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

CarPosition State::getCarById(long long id) const {
    auto car = find_if(cars.begin(), cars.end(), [&id](const CarPosition& it) {
        return it.original->getId() == id;
    });
    if (car == cars.end()) {
        cerr << "car with id " << id << " not found" << endl;
        return cars.front();
    }
    return *car;
}
