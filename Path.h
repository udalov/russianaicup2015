#pragma once

#include "math2d.h"

#include <string>
#include <vector>

using std::string;
using std::vector;

struct Tile {
    int x;
    int y;

    Tile(): x(), y() { }
    Tile(int x, int y): x(x), y(y) { }
    Tile(const Tile& other): x(other.x), y(other.y) { }

    Tile *operator=(const Tile& other) {
        x = other.x;
        y = other.y;
        return this;
    }

    bool operator==(const Tile& other) const {
        return x == other.x && y == other.y;
    }

    bool isNeighborTo(const Tile& other) const;

    Point toPoint() const;

    string toString() const;
};

vector<Tile> bestPath(const Tile& start, const Tile& finish);
