#include "Go.h"

#include <sstream>

using namespace std;

void Go::applyTo(Move& move) const {
    move.setEnginePower(enginePower);
    move.setWheelTurn(wheelTurn);
    move.setBrake(brake);
    move.setThrowProjectile(throwProjectile);
    move.setUseNitro(useNitro);
    move.setSpillOil(spillOil);
}

string Go::toString() const {
    ostringstream ss;
    ss.precision(8);
    ss << fixed << "(engine " << enginePower << " wheels " << wheelTurn;
    if (brake) ss << " BRAKE";
    if (throwProjectile) ss << " FIRE";
    if (useNitro) ss << " NITRO";
    if (spillOil) ss << " OIL";
    ss << ")";
    return ss.str();
}
