#pragma once

#include "model/Game.h"

using model::Game;

class Const {
public:
    Game game;

    static Const& getInstance();
    static Game& getGame();
private:
    Const() { }
    Const(const Const& other) = delete;
    Const *operator=(const Const& other) = delete;
};
