#pragma once

#include "math2d.h"

#include <functional>
#include <string>
#include <vector>

using std::hash;
using std::string;
using std::vector;

struct Tile {
    int x;
    int y;

    Tile(): x(), y() { }
    explicit Tile(int x, int y): x(x), y(y) { }
    explicit Tile(const Point& point);

    Tile *operator=(const Tile& other);

    bool operator==(const Tile& other) const;
    bool operator!=(const Tile& other) const;

    size_t manhattanDistanceTo(const Tile& other) const;
    bool isNeighborTo(const Tile& other) const;

    Point toPoint() const;

    string toString() const;
};

struct DirectedTile {
    Tile tile;
    int direction; // 0..7

    explicit DirectedTile(const Tile& tile, int direction): tile(tile), direction(direction) { }

    bool operator==(const DirectedTile& other) const;

    DirectedTile turnLeft() const;
    DirectedTile turnRight() const;

    string toString() const;
};

namespace std {
    template<> struct hash<DirectedTile> {
        size_t operator()(const DirectedTile& dt) const;
    };
}

ostream& operator<<(ostream& out, const Tile& tile);
ostream& operator<<(ostream& out, const DirectedTile& tile);

vector<DirectedTile> bestPath(const DirectedTile& start, const Tile& finish);
