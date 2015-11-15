#pragma once

#include "math2d.h"
#include "model/Car.h"
#include "model/OilSlick.h"
#include "model/Projectile.h"
#include "model/World.h"
#include "Go.h"

#include <vector>

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
    int nitroCharges;
    int nitroCooldown;

    CarPosition(const Car *original, const Point& location, const Vec& velocity, double angle, double angularSpeed,
                double enginePower, double wheelTurn, double health, int nitroCharges, int nitroCooldown)
            : original(original), location(location), velocity(velocity), angle(angle), angularSpeed(angularSpeed),
              enginePower(enginePower), wheelTurn(wheelTurn), health(health), nitroCharges(nitroCharges),
              nitroCooldown(nitroCooldown) { }

    CarPosition(const Car *car)
            : original(car),
              location(Point(car->getX(), car->getY())),
              velocity(Vec(car->getSpeedX(), car->getSpeedY())),
              angle(car->getAngle()),
              angularSpeed(car->getAngularSpeed()),
              enginePower(car->getEnginePower()),
              wheelTurn(car->getWheelTurn()),
              health(car->getDurability()),
              nitroCharges(car->getNitroChargeCount()),
              nitroCooldown(car->getRemainingNitroCooldownTicks()) { }

    Vec direction() const { return Vec(angle); }

    void advance(const Go& move, double medianAngularSpeed, double updateFactor);

    Rectangle rectangle() const;

    string toString() const;
};

struct OilSlickPosition {
    const OilSlick *original;
    int remainingTime;

    OilSlickPosition(const OilSlick *original, int remainingTime)
            : original(original), remainingTime(remainingTime) { }

    OilSlickPosition(const OilSlick *oilSlick)
            : original(oilSlick),
              remainingTime(oilSlick->getRemainingLifetime()) { }

    OilSlickPosition apply() const;
};

struct WasherPosition {
    const Projectile *original;
    Point location;
    Vec velocity;

    WasherPosition(const Projectile *original, const Point& location, const Vec& velocity)
            : original(original), location(location), velocity(velocity) { }

    WasherPosition(const Projectile *washer)
            : original(washer),
              location(Point(washer->getX(), washer->getY())),
              velocity(Vec(washer->getSpeedX(), washer->getSpeedY())) { }

    WasherPosition apply() const;
};

struct State {
    const World *original;
    vector<CarPosition> cars;
    vector<OilSlickPosition> oilSlicks;
    vector<WasherPosition> washers;

    State(const World *original, const vector<CarPosition>& cars, const vector<OilSlickPosition>& oilSlicks,
          const vector<WasherPosition>& washers)
            : original(original), cars(cars), oilSlicks(oilSlicks), washers(washers) { }

    State(const State& other) : original(other.original), cars(other.cars), oilSlicks(other.oilSlicks),
                                washers(other.washers) { }

    State(const World *world);

    CarPosition getCarById(long long id) const;

    State apply(const vector<Go>& moves) const;
};
