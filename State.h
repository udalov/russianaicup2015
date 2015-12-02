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
using std::string;
using std::vector;

struct CarPosition {
    const Car *original;
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

    CarPosition(const Car *car);

    Vec direction() const { return Vec(angle); }

    void advance(const Go& move);

    bool isBuggy() const;

    Point bumperCenter() const;
    Tile tile() const;
    DirectedTile directedTile() const;

    string toString() const;
};

struct OilSlickPosition {
    const OilSlick *original;
    int remainingTime;

    OilSlickPosition(const OilSlick *oilSlick)
            : original(oilSlick),
              remainingTime(oilSlick->getRemainingLifetime()) { }

    bool isAlive() const;
    void apply();
};

struct WasherPosition {
    const Projectile *original;
    Point location;
    Vec velocity;

    WasherPosition(const Projectile *washer)
            : original(washer),
              location(Point(washer->getX(), washer->getY())),
              velocity(Vec(washer->getSpeedX(), washer->getSpeedY())) { }

    void apply();
};

struct BonusPosition {
    const Bonus *original;
    Point location;
    Rect outerRectangle;
    Rect innerRectangle;
    bool isAlive;

    BonusPosition(const Bonus *bonus);
};

struct State {
    const World *original;
    vector<CarPosition> cars;
    vector<OilSlickPosition> oilSlicks;
    vector<WasherPosition> washers;
    vector<BonusPosition> bonuses;

    State(const World *world);

    const CarPosition& me() const;

    void apply(const Go& move);
};
