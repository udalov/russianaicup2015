#include "Map.h"

using namespace model;

char getEdgesByTileType(TileType type) {
    switch (type) {
        case EMPTY: return 0b0000;
        case VERTICAL: return 0b1010;
        case HORIZONTAL: return 0b0101;
        case LEFT_TOP_CORNER: return 0b0011;
        case RIGHT_TOP_CORNER: return 0b0110;
        case LEFT_BOTTOM_CORNER: return 0b1001;
        case RIGHT_BOTTOM_CORNER: return 0b1100;
        case LEFT_HEADED_T: return 0b1110;
        case RIGHT_HEADED_T: return 0b1011;
        case TOP_HEADED_T: return 0b1101;
        case BOTTOM_HEADED_T: return 0b0111;
        case CROSSROADS: return 0b1111;
        case _UNKNOWN_TILE_TYPE_: case _TILE_TYPE_COUNT_:;
    }

    return 0;
}

void Map::update(const World& world) {
    auto tiles = world.getTilesXY();

    if (graph.empty()) {
        width = tiles.size();
        height = tiles[0].size();
        graph.resize(width, vector<char>(height));
    }

    for (unsigned long i = 0; i < width; i++) {
        for (unsigned long j = 0; j < height; j++) {
            graph[i][j] = getEdgesByTileType(tiles[i][j]);
        }
    }
}
