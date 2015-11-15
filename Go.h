#pragma once

#include <string>

using std::string;

struct Go {
    double enginePower;
    double wheelTurn;
    // TODO: pack
    bool brake;
    bool throwProjectile;
    bool useNitro;
    bool spillOil;

    Go(double enginePower,
       double wheelTurn,
       bool brake = false,
       bool throwProjectile = false,
       bool useNitro = false,
       bool spillOil = false)
            : enginePower(enginePower), wheelTurn(wheelTurn), brake(brake), throwProjectile(throwProjectile),
              useNitro(useNitro), spillOil(spillOil) { }

    string toString() const;
};
