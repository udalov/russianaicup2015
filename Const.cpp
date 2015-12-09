#include "Const.h"

Const& Const::getInstance() {
    static Const instance;
    return instance;
}

Game& Const::getGame() {
    return getInstance().game;
}
