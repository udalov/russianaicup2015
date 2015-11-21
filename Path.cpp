#include "Path.h"
#include "Const.h"
#include "Map.h"

#include <cmath>
#include <iostream>
#include <sstream>
#include <unordered_map>

using namespace std;

bool Tile::isNeighborTo(const Tile& other) const {
    return abs(x - other.x) + abs(y - other.y) < 1;
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

vector<Tile> bestPath(const Tile& start, const Tile& finish) {
    static const int dx[] = {1, 0, -1, 0};
    static const int dy[] = {0, 1, 0, -1};
    auto& map = Map::getMap();

    if (start == finish) return vector<Tile>(1, finish);
    
    int startEnc = (start.x << 8) + start.y;
    int finishEnc = (finish.x << 8) + finish.y;

    vector<int> q;
    unordered_map<int, int> prev;
    q.push_back(startEnc);
    prev[startEnc] = -1;
    unsigned long qb = 0;
    while (qb < q.size()) {
        int v = q[qb++];
        int xx = v >> 8, yy = v & 255;
        if (v == finishEnc) {
            vector<Tile> result;
            while (prev.find(v) != prev.end()) {
                result.emplace_back(v >> 8, v & 255);
                if (prev[v] == startEnc) break;
                v = prev[v];
            }
            reverse(result.begin(), result.end());
            return result;
        }
        for (int d = 0; d < 4; d++) {
            if (map.graph[xx][yy] & (1 << d)) {
                int nx = xx + dx[d], ny = yy + dy[d];
                int nv = (nx << 8) + ny;
                if (prev.find(nv) == prev.end()) {
                    prev[nv] = v;
                    q.push_back(nv);
                }
            }
        }
    }
    
    cerr << "path not found from " << start.toString() << " to " << finish.toString() << endl;
    return vector<Tile>(1, finish);
}
