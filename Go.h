#pragma once

#include "model/Car.h"
#include "model/Move.h"

#include <string>
#include <vector>

using model::Car;
using model::Move;
using std::string;
using std::vector;

enum class WheelTurnDirection {
    TURN_LEFT,
    KEEP,
    TURN_RIGHT,
};

struct Go {
    double enginePower;
    WheelTurnDirection wheelTurn;
    // TODO: pack
    bool brake;
    bool throwProjectile;
    bool useNitro;
    bool spillOil;

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
