#pragma once

#include "model/Game.h"

using model::Game;

struct Const {
    Game game;

    Const() { }

    static Const& getInstance() {
        static Const instance;
        return instance;
    }

    static Game& getGame() {
        return getInstance().game;
    }
};
