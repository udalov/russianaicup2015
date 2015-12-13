#pragma once

#include "model/World.h"

#include <vector>

using model::World;
using std::vector;

class Map {
public:
    size_t width;
    size_t height;
    int hashCode;
    int waypointsHash;

    static Map& getMap();

    // Each element is a bit mask of edges to (+1, 0) [bit #0], (0, +1), (-1, 0), (0, -1) [bit #3]
    // TODO: return 0 on out of bounds
    int get(size_t x, size_t y) const;
    void set(size_t x, size_t y, int value);

    bool isApproximated(size_t x, size_t y) const;

    void update(const World& world);

private:
    vector<char> graph;
    vector<vector<int>> waypoints;

    Map() { }
    Map(const Map& other) = delete;
    Map *operator=(const Map& other) = delete;
};
