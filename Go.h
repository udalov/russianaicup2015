#pragma once

#include "model/Car.h"
#include "model/Move.h"

#include <iostream>
#include <string>
#include <vector>

using model::Car;
using model::Move;
using std::ostream;
using std::string;
using std::vector;

enum class WheelTurnDirection {
    TURN_LEFT,
    KEEP,
    TURN_RIGHT,
};

struct Go {
    double enginePower;
    WheelTurnDirection wheelTurn : 4;
    bool brake : 1;
    bool throwProjectile : 1;
    bool useNitro : 1;
    bool spillOil : 1;

    Go() : enginePower(), wheelTurn(WheelTurnDirection::KEEP), brake(), throwProjectile(), useNitro(), spillOil() { }

    Go(double enginePower,
       WheelTurnDirection wheelTurn,
       bool brake = false,
       bool throwProjectile = false,
       bool useNitro = false,
       bool spillOil = false)
            : enginePower(enginePower), wheelTurn(wheelTurn), brake(brake), throwProjectile(throwProjectile),
              useNitro(useNitro), spillOil(spillOil) { }

    bool operator==(const Go& other) const;

    void applyTo(const Car& car, Move& move) const;

    string toString() const;
};

ostream& operator<<(ostream& out, const Go& move);
