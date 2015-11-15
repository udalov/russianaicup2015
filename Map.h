#pragma once

#include "model/World.h"

#include <vector>

using model::World;
using std::vector;

class Map {
public:
    unsigned long width;
    unsigned long height;

    // Each element is a bit mask of edges to (+1, 0) [bit #0], (0, +1), (-1, 0), (0, -1) [bit #3]
    vector<vector<char>> graph;

    static Map& getMap() {
        static Map instance;
        return instance;
    }

    void update(const World& world);

private:
    Map() { }
    Map(const Map& other) = delete;
    Map *operator=(const Map& other) = delete;
};
