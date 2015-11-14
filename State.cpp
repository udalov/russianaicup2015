#include "State.h"
#include "Const.h"

#include <algorithm>
#include <sstream>
#include <iostream>

using namespace std;

const int ITERATION_COUNT_PER_STEP = 10;
const double EPSILON = 1e-7;

State::State(const World *world) : original(world) {
    auto& cars = world->getCars();
    this->cars.reserve(cars.size());
    for (unsigned long i = 0, size = cars.size(); i < size; i++) {
        this->cars.push_back(CarPosition(&cars[i]));
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

State State::apply(const vector<Go>& moves) const {
    const double updateFactor = 1.0 / ITERATION_COUNT_PER_STEP;

    vector<CarPosition> cars(this->cars);
    vector<double> medianAngularSpeed(cars.size());
    for (unsigned long i = 0, size = cars.size(); i < size; i++) {
        cars[i].enginePower = updateEnginePower(cars[i].enginePower, moves[i].enginePower);
        cars[i].wheelTurn = updateWheelTurn(cars[i].wheelTurn, moves[i].wheelTurn);
        cars[i].angularSpeed =
                cars[i].wheelTurn * Const::getGame().getCarAngularSpeedFactor() *
                (cars[i].velocity * cars[i].direction());

        medianAngularSpeed[i] = cars[i].angularSpeed;
    }
    for (int iteration = 1; iteration <= ITERATION_COUNT_PER_STEP; iteration++) {
        for (unsigned long i = 0, size = cars.size(); i < size; i++) {
            cars[i].advance(moves[i], medianAngularSpeed[i], updateFactor);
        }
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

vector<Point> CarPosition::getPoints() const {
    auto forward = Vec(angle) * (original->getWidth() / 2);
    auto sideways = Vec(angle + M_PI / 2) * (original->getHeight() / 2);
    vector<Point> result {
            location + forward - sideways,
            location + forward + sideways,
            location - forward + sideways,
            location - forward - sideways
    };
    return result;
}

string CarPosition::toString() const {
    ostringstream ss;
    ss.precision(8);
    ss << fixed << "car at " << location.toString() <<
            " going " << velocity.toString() <<
            " angle " << angle <<
            " angular " << angularSpeed <<
            " engine " << enginePower <<
            " wheel " << wheelTurn;
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
