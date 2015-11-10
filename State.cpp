#include "State.h"

State::State(const World *world) {
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

State State::apply(const vector<Go>& moves) const {
    auto cars = vector<CarPosition> { this->cars };
    for (auto i = 0UL, size = cars.size(); i < size; i++) {
        cars[i] = cars[i].apply(moves[i]);
    }

    auto oilSlicks = vector<OilSlickPosition> { };
    for (auto& oilSlick : this->oilSlicks) {
        if (oilSlick.remainingTime > 0 /* or 1? */) {
            oilSlicks.push_back(oilSlick.apply());
        }
    }

    auto washers = vector<WasherPosition> { };
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

CarPosition CarPosition::apply(const Go& move) const {
    // TODO
    return CarPosition(this->original);
}
