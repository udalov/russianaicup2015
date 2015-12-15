#include "Go.h"

#include <sstream>

using namespace std;

bool Go::operator==(const Go& other) const {
    return enginePower == other.enginePower &&
            wheelTurn == other.wheelTurn &&
            brake == other.brake &&
            throwProjectile == other.throwProjectile &&
            useNitro == other.useNitro &&
            spillOil == other.spillOil;
}

void Go::applyTo(const Car& car, Move& move) const {
    move.setEnginePower(enginePower);

    switch (wheelTurn) {
        case WheelTurnDirection::TURN_LEFT: move.setWheelTurn(-1.0); break;
        case WheelTurnDirection::KEEP: move.setWheelTurn(car.getWheelTurn()); break;
        case WheelTurnDirection::TURN_RIGHT: move.setWheelTurn(1.0); break;
    }

    move.setBrake(brake);
    move.setThrowProjectile(throwProjectile);
    move.setUseNitro(useNitro);
    move.setSpillOil(spillOil);
}

string Go::toString() const {
    ostringstream ss;
    ss.precision(3);
    ss << fixed << "(" << enginePower << " ";
    switch (wheelTurn) {
        case WheelTurnDirection::TURN_LEFT: ss << "TURN_LEFT"; break;
        case WheelTurnDirection::KEEP: ss << "KEEP"; break;
        case WheelTurnDirection::TURN_RIGHT: ss << "TURN_RIGHT"; break;
    }
    if (brake) ss << " BRAKE";
    if (throwProjectile) ss << " FIRE";
    if (useNitro) ss << " NITRO";
    if (spillOil) ss << " OIL";
    ss << ")";
    return ss.str();
}

ostream& operator<<(ostream& out, const Go& move) {
    out << move.toString();
    return out;
}
