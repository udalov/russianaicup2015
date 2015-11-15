#pragma once

#include "model/Game.h"

using model::Game;

class Const {
public:
    Game game;

    static Const& getInstance() {
        static Const instance;
        return instance;
    }

    static Game& getGame() {
        return getInstance().game;
    }

private:
    Const() { }
    Const(const Const& other) = delete;
    Const *operator=(const Const& other) = delete;
};
