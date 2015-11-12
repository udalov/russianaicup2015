#include "State.h"
#include "Const.h"

#include <sstream>

using namespace std;

const int ITERATION_COUNT_PER_STEP = 10;

State::State(const World *world) : original(world) {
    auto& cars = world->getCars();
    this->cars.reserve(cars.size());
    for (auto i = 0UL, size = cars.size(); i < size; i++) {
        this->cars.push_back(CarPosition(&cars[i]));
    }

    auto& oilSlicks = world->getOilSlicks();
    this->oilSlicks.reserve(oilSlicks.size());
    for (auto i = 0UL, size = oilSlicks.size(); i < size; i++) {
        this->oilSlicks.push_back(OilSlickPosition(&oilSlicks[i]));
    }

    auto& washers = world->getProjectiles();
    for (auto i = 0UL, size = washers.size(); i < size; i++) {
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
    vector<double> newEnginePower(cars.size());
    vector<double> newWheelTurn(cars.size());
    for (auto i = 0UL, size = cars.size(); i < size; i++) {
        newEnginePower[i] = updateEnginePower(cars[i].enginePower, moves[i].enginePower);
        newWheelTurn[i] = updateWheelTurn(cars[i].wheelTurn, moves[i].wheelTurn);
    }
    for (auto i = 0UL, size = cars.size(); i < size; i++) {
        // TODO: this formula is not correct, this should be a median angular velocity instead
        cars[i].angularSpeed +=
                newWheelTurn[i] * Const::getGame().getCarAngularSpeedFactor() *
                (cars[i].velocity * Vec(cars[i].angle));
    }
    for (int iteration = 1; iteration <= ITERATION_COUNT_PER_STEP; iteration++) {
        for (auto i = 0UL, size = cars.size(); i < size; i++) {
            cars[i] = cars[i].apply(moves[i], newEnginePower[i], newWheelTurn[i], updateFactor);
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

CarPosition CarPosition::apply(const Go& move, double newEnginePower, double newWheelTurn, double updateFactor) const {
    auto game = Const::getGame();

    // Location

    auto newLocation = location.shift(velocity * updateFactor);

    auto acceleration = newEnginePower *
            (newEnginePower < 0 ? game.getBuggyEngineRearPower() : game.getBuggyEngineForwardPower()) /
            game.getBuggyMass();
    auto newVelocity = velocity + Vec(angle) * acceleration * updateFactor;

    // Air friction

    auto airFriction = pow(1.0 - game.getCarMovementAirFrictionFactor(), updateFactor);
    newVelocity = newVelocity * airFriction;

    // Movement friction

    auto lengthwiseVelocityChange = game.getCarLengthwiseMovementFrictionFactor() * updateFactor;
    auto lengthwiseUnitVector = Vec(angle);
    auto lengthwiseVelocityPart = newVelocity * lengthwiseUnitVector;
    lengthwiseVelocityPart =
            lengthwiseVelocityPart >= 0.0
            ? max(lengthwiseVelocityPart - lengthwiseVelocityChange, 0.0)
            : min(lengthwiseVelocityPart + lengthwiseVelocityChange, 0.0);

    auto crosswiseVelocityChange = game.getCarCrosswiseMovementFrictionFactor() * updateFactor;
    auto crosswiseUnitVector = Vec(angle + M_PI / 2);
    auto crosswiseVelocityPart = newVelocity * crosswiseUnitVector;
    crosswiseVelocityPart =
            crosswiseVelocityPart >= 0.0
            ? max(crosswiseVelocityPart - crosswiseVelocityChange, 0.0)
            : min(crosswiseVelocityPart + crosswiseVelocityChange, 0.0);

    newVelocity = lengthwiseUnitVector * lengthwiseVelocityPart + crosswiseUnitVector * crosswiseVelocityPart;

    // Angle

    auto newAngle = angle + angularSpeed * updateFactor;

    // Rotation air friction

    auto rotationAirFriction = pow(1.0 - game.getCarRotationAirFrictionFactor(), updateFactor);
    auto newAngularSpeed = angularSpeed * rotationAirFriction;

    // Rotation friction

    if (abs(newAngularSpeed) > 0.0) {
        auto rotationFrictionFactor = game.getCarRotationFrictionFactor() * updateFactor;
        newAngularSpeed =
                rotationFrictionFactor >= abs(newAngularSpeed) ? 0.0 : newAngularSpeed - rotationFrictionFactor;
    }

    return CarPosition(this->original, newLocation, newVelocity, newAngle, newAngularSpeed, newEnginePower,
                       newWheelTurn, this->health);
}

vector<Point> CarPosition::getPoints() const {
    auto forward = Vec(angle) * (original->getWidth() / 2);
    auto sideways = Vec(angle + M_PI / 2) * (original->getHeight() / 2);
    return {
            location.shift(forward - sideways),
            location.shift(forward + sideways),
            location.shift(-forward + sideways),
            location.shift(-forward - sideways)
    };
}

string CarPosition::toString() const {
    ostringstream ss;
    ss << "car at " << location.toString() <<
            " going " << velocity.toString() <<
            " angle " << to_string(angle) <<
            " angular " << to_string(angularSpeed) <<
            " engine " << to_string(enginePower) <<
            " wheel " << to_string(wheelTurn);
    return ss.str();
}
