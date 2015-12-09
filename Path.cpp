#include "Path.h"
#include "Const.h"
#include "Debug.h"
#include "Map.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <sstream>
#include <unordered_map>

using namespace std;

Tile::Tile(const Point& point) {
    static const double tileSize = Const::getGame().getTrackTileSize();
    static const int widthM1 = Map::getMap().width - 1;
    static const int heightM1 = Map::getMap().height - 1;

    x = max(min(static_cast<int>(point.x / tileSize), widthM1), 0);
    y = max(min(static_cast<int>(point.y / tileSize), heightM1), 0);
}

Tile *Tile::operator=(const Tile& other) {
    x = other.x;
    y = other.y;
    return this;
}

bool Tile::operator==(const Tile& other) const {
    return x == other.x && y == other.y;
}

bool Tile::operator!=(const Tile& other) const {
    return x != other.x || y != other.y;
}

bool DirectedTile::operator==(const DirectedTile& other) const {
    return tile == other.tile && direction == other.direction;
}

DirectedTile DirectedTile::turnLeft() const {
    return DirectedTile(tile, direction == 0 ? 7 : direction - 1);
}

DirectedTile DirectedTile::turnRight() const {
    return DirectedTile(tile, direction == 7 ? 0 : direction + 1);
}

unsigned int Tile::manhattanDistanceTo(const Tile& other) const {
    return abs(x - other.x) + abs(y - other.y);
}

bool Tile::isNeighborTo(const Tile& other) const {
    return manhattanDistanceTo(other) == 1;
}

Point Tile::toPoint() const {
    static double trackTileSize = Const::getGame().getTrackTileSize();
    return Point((x + 0.5) * trackTileSize, (y + 0.5) * trackTileSize);
}

string Tile::toString() const {
    ostringstream ss;
    ss << "[" << x << "," << y << "]";
    return ss.str();
}

string DirectedTile::toString() const {
    ostringstream ss;
    ss << "[" << tile.x << "," << tile.y << "," << direction << "]";
    return ss.str();
}

namespace std {
    size_t hash<DirectedTile>::operator()(const DirectedTile& dt) const {
        return (dt.tile.x << 16) | (dt.tile.y << 8) | dt.direction;
    }
}

vector<DirectedTile> bestPath(const DirectedTile& start, const Tile& finish) {
    static const int dx[] = {1, 0, -1, 0};
    static const int dy[] = {0, 1, 0, -1};
    auto& map = Map::getMap();

    if (start.tile == finish) return { start };
    
    vector<DirectedTile> q;
    unordered_map<DirectedTile, DirectedTile> prev;
    q.push_back(start);
    unsigned long qb = 0;
    while (qb < q.size()) {
        auto v = q[qb++];
        int xx = v.tile.x, yy = v.tile.y;
        if (v.tile == finish) {
            vector<DirectedTile> result;
            result.reserve(start.tile.manhattanDistanceTo(finish) + 1); // TODO
            while (prev.find(v) != prev.end()) {
                result.push_back(v);
                auto w = prev.find(v);
                if (w == prev.end()) {
                    cerr << "bad path: " << start.toString() << " to " << finish.toString() << endl;
                    break;
                }
                if (w->second == start) break;
                v = w->second;
            }
            result.push_back(start);
            reverse(result.begin(), result.end());
            return result;
        }

        for (int dd = v.direction >> 1; dd <= (v.direction + 1) >> 1; dd++) {
            int d = dd == 4 ? 0 : dd;
            if (map.get(xx, yy) & (1 << d)) {
                int nx = xx + dx[d], ny = yy + dy[d];
                auto next = DirectedTile(Tile(nx, ny), v.direction);
                if (Debug::isMap20) { // :(
                    if (xx == 5 && yy == 4 && nx == 4 && ny == 4) continue;
                    if (xx == 13 && yy == 2 && nx == 12 && ny == 2) continue;
                    if (xx == 9 && yy == 10 && nx == 10 && ny == 10) continue;
                    if (xx == 1 && yy == 12 && nx == 2 && ny == 12) continue;
                }
                if (prev.find(next) == prev.end()) {
                    prev.insert({ next, v });
                    q.push_back(next);
                }
            }
        }

        for (auto& next : { v.turnLeft(), v.turnRight() }) {
            if (prev.find(next) == prev.end()) {
                prev.insert({ next, v });
                q.push_back(next);
            }
        }
    }
    
    cerr << "path not found from " << start.toString() << " to " << finish.toString() << endl;
    return { start, DirectedTile(finish, start.direction) };
}
