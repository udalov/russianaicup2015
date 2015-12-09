#pragma once

#include "model/World.h"

#include <vector>

using model::World;
using std::vector;

class Map {
public:
    unsigned long width;
    unsigned long height;
    int hashCode;
    int waypointsHash;

    static Map& getMap();

    // Each element is a bit mask of edges to (+1, 0) [bit #0], (0, +1), (-1, 0), (0, -1) [bit #3]
    // TODO: return 0 on out of bounds
    int get(unsigned long x, unsigned long y) const;
    void set(unsigned long x, unsigned long y, int value);

    void update(const World& world);

private:
    vector<char> graph;
    vector<vector<int>> waypoints;

    Map() { }
    Map(const Map& other) = delete;
    Map *operator=(const Map& other) = delete;
};
