#pragma once

#include "math2d.h"
#include "model/Bonus.h"
#include "model/Car.h"
#include "model/OilSlick.h"
#include "model/Projectile.h"
#include "model/World.h"
#include "Go.h"
#include "Path.h"

#include <vector>

using model::Bonus;
using model::Car;
using model::OilSlick;
using model::Projectile;
using model::World;
using std::move;
using std::string;
using std::vector;

struct CarPosition {
    const Car *original;
    const World *originalWorld;
    Point location;
    Vec velocity;
    double angle;
    double angularSpeed;
    double enginePower;
    double wheelTurn;
    double health;
    int projectiles;
    int nitroCharges;
    int oilCanisters;
    int nitroCooldown;
    Rect rectangle;

    int medicines;
    int pureScore;

    explicit CarPosition(const World *world, const Car *car);

    Vec direction() const { return Vec(angle); }

    void advance(const Go& move);

    bool isBuggy() const;

    Point bumperCenter() const;
    Tile tile() const;
    DirectedTile directedTile() const;

    string toString() const;
};

struct BonusPosition {
    const Bonus *original;
    Point location;
    Rect innerRectangle;
    bool isAlive;

    explicit BonusPosition(const Bonus *bonus);
};

struct State {
    const World *original;
    CarPosition car;
    vector<BonusPosition> bonuses;
    vector<double> bonusX; // X coords of bonuses

    explicit State(const World *world, long long id);

    const CarPosition& me() const { return car; }

    void apply(const Go& move);
};
