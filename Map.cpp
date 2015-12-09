#include "Map.h"
#include "Debug.h"

#include <iostream>
#include <sstream>
#include <string>
#include <unordered_map>

using namespace model;
using namespace std;

//#define PRINT_MAP

unordered_map<int, string> PRECALCULATED_MAPS = {
{ 2072916160, "773aaaaaa95000000550000005500000055000000550000005500000056aaaaaac" },
{ 1116265582, "7700003aa900005005000050053aaafaac5000500050005000500050006aaac000" },
{ 1762890787, "773aa93aa950055005500550056aaffaac3aaffaa950055005500550056aac6aac" },
{ 87035876, "77003a900003c069003c003ea97aaac0056900000506900005006baabc0006aac0" },
{ -747552961, "993a93a900005055050000506fac0000500500000050050003a9500690050550006aafac50000006a9690000000506aaaaaaac" },
{ 844785447, "8e3aaaaaaaaa903a9500000000050505503aaaaaaac050550500000000050550503aaaaa90505505050000050505506ac03a906ac055000005050000056aaaaac06aaaaac" },
{ -886070406, "ff3bbbbbbbbbbbbbb956eeeeeeeeeeeeed6bbaaaaaaaabba9506eaaaaaaaaec3d500000000000007d500000000000007d500000000000007c50000000000003e9500000000000050550000000000006bd500000000000007d500000000000007d500000000000007d50000000000003ec50000000000007abc0000000000006ac0" },
{ 999268544, "ff00000000003aba900000000003c3c069000000003c3c000500000003c3c000050000003c3c000005000003c3c000000500003c3c000000050003c3c000000005003c3c000000000503c3c000000000053c3c00000000000553c00000000000057c000000000000055000000000000005690000000000003c06aaaaaaaaaaaac0" },
{ 752098369, "bb03aaaabaaaa93c0003c0000550000500000550003c0003ad5003c000050553ac000005057c00000005055000000005055003aaa906ad5005000500055005000500056aac0006aaac" },
{ 363547794, "bb3abbbbb90000507fffffaba96aeeeeec05050000000005050000000005053aaa90000505500050000505503ac0000505505000000505506aaaaaac055000000000056aaaaaaaaaac" },
{ -287746337, "ff3aa9003aa9003aa9500500500500500550050050050050056aafaafaafaafaac000500500500500000050050050050003aafaaeaafaaeaa9500500000500000550050000050000056aafaa9006aa9005000500500000500500050050000050053aafaac003aafaac500500000500500050050000050050006aac000006aac000" },
{ -1743023107, "883aaaaaa9053aaaa950553aa95505553955505555555506ffefeff9056afac5506aafaac500006aaac" },
{ -496608672, "993a900003905069003c69690693c03c06907d03c0006bc6bc00003e93e90003c07d06903c03c69069693c00690506c00006ac" },
{ 104538592, "ee000003aba90000000003c0506900000003c03e9069000003c03c0690690003c03c0006906903c03c0000069069503c000000069057ad0000000007ad506900000003c0569069000003c03c0690690003c03c0006906903c03c000006906bc03c0000000690503c0000000006aeac00000" },
{ -141758944, "cc3aa9000003a90500690003c069690050005000506907903d003c006bffaffbac00006fc06fc00000005000500000003f903f900003aeffaffe9003c007c06d069050005000500696903c0006900506ac000006aac" },
{ 1594662563, "883aaabaaa95000500055000500055000500057aaafaaad5000500055000500055000500056aaaeaaac" },
{ 2125229244, "ff003aaaabaaaa900003c00005000050003e903bbfbbbbfbb979507ffffffffffd7d507ffffffffffd6d507ffffffffffd3d507ffffffffffd7d507ffffffffffd7d507ffffffffffd6d507ffffffffffd3d507ffffffffffd7d507ffffffffffd7c507ffffffffffd6bc06eeeeeeefeec0690000000005000006aaaaaaaaac000" },
{ 754499905, "ff03aaaaaaaaaaaaa905000000000000053fbaaaaaaaaaaaac7fd00000000000007fd000003a9000007ed0000050500000505000007bd000007bd000007fd000007fe900007fd000007c0500006fd00000503c000007d00000505000003fd000007be90003ffd000006d07bbbfffc0000006bffefffc000000006ec06ec0000000" },
{ -1196455207, "ff03aaaaaabaaba9003c000000500506905003aa906aac0069503c005000000005506aa96aaaa900055000050000069005500005000000693d790006abaa9006ec7f900005005000007ee90006aaeaaaa95005000000000005500500000000000579053bbbb90000056eafeeefd7baaaac00050007d7c000000006aaaeec000000" },
{ -948980063, "77003a9000005050003afac3a95050050550500505506aafac500005006aaaac00" },
{ 1705052841, "ee0000000000003a900000000000050500003aaaaaaafac000050000000500003afa90000050000505050000050000506ac00000500005000000000500005000003a90500005000005050500005000006afac000050000000500003afaaaaaaac00005050000000000006ac000000000000" },
{ 271034593, "aa3aaa903a90050005050500503ac06afa950500000505506a903afac50005050500503ac06afa950500000505506aaaaafac500000005006aaaaaaac00" },
{ -772099544, "f703aaa9000500050005003c000503c00005069000050069003fa9069055050069550500056fad003c050503c005053c000507c00005050000053c000006c00000" },
{ -639158449, "f73ababba9507aed0550500505505005056be93ebc0505505007bd7bd007fd7fd006fc6fc00050050003e93e900505505007bd7bd006fc6fc000500500006aac00" },
{ 424037989, "ff0003bbb900000000003feeff9000000003fc396ffbbbaa903fc3fe96eeed00507d3fc07bbb9500507c6fb96eeed6abd07b96ffbbb96b96f96ff96eeeef97e97d06ff900007d5057d006fd00007d5057d003fc3bbbfd5056d03fc07eeeec6bd053fc007903b907c3d7c0006d3feeac3fc53bbbbc6fbbbbfc06eeeec006eeeec00" },
};

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
        case UNKNOWN: return 0b1111;
        case _UNKNOWN_TILE_TYPE_: case _TILE_TYPE_COUNT_:;
    }

    return 0;
}

char approximateUnknownTile(int i, int j, int width, int height) {
    static const int dx[] = {1, 0, -1, 0};
    static const int dy[] = {0, 1, 0, -1};

    char result = 0;
    for (int d = 0; d < 4; d++) {
        int x = i + dx[d], y = j + dy[d];
        if (0 <= x && x < width && 0 <= y && y < height) {
            result |= (1 << d);
        }
    }

    return result;
}

Map& Map::getMap() {
    static Map *instance = new Map();
    return *instance;
}

int Map::get(unsigned long x, unsigned long y) const {
    // return x < width && y < height ? static_cast<int>(graph[x * height + y]) : 0;
    return static_cast<int>(graph[x * height + y]);
}

void Map::set(unsigned long x, unsigned long y, int value) {
    graph[x * height + y] = static_cast<char>(value);
}

#ifdef PRINT_MAP
string serializeMap(const Map& map) {
    stringstream ss;
    ss << hex << (map.width - 1) << (map.height - 1);
    for (unsigned long i = 0; i < map.width; i++) {
        for (unsigned long j = 0; j < map.height; j++) {
            ss << map.get(i, j);
        }
    }
    return ss.str();
}
#endif

int fromHex(char c) {
    stringstream ss;
    ss << c;
    int decimal;
    ss >> hex >> decimal;
    return decimal;
}

bool deserializeMap(Map& map, const string& data) {
    unsigned long n = map.width;
    unsigned long m = map.height;
    if (fromHex(data[0]) != static_cast<int>(n - 1) ||
        fromHex(data[1]) != static_cast<int>(m - 1)) return false;

    for (unsigned long i = 0; i < n; i++) {
        for (unsigned long j = 0; j < m; j++) {
            auto previous = map.get(i, j);
            if (previous != static_cast<char>(-1) && previous != fromHex(data.at(i * m + j + 2))) return false;
        }
    }

    for (unsigned long i = 0; i < n; i++) {
        for (unsigned long j = 0; j < m; j++) {
            map.set(i, j, fromHex(data.at(i * m + j + 2)));
        }
    }

    return true;
}

void Map::update(const World& world) {
    auto& tiles = world.getTilesXY();

    if (graph.empty()) {
        width = tiles.size();
        height = tiles[0].size();
        graph.resize(width * height, static_cast<char>(-1));
    }

    bool allTilesKnown = true;
    hashCode = 0;
    for (unsigned long i = 0; i < width; i++) {
        for (unsigned long j = 0; j < height; j++) {
            auto tile = tiles[i][j];
            int previousValue = get(i, j);
            if (tile == UNKNOWN) {
                if (previousValue == static_cast<char>(-1)) {
                    set(i, j, approximateUnknownTile(i, j, width, height));
                }
            } else {
                int value = getEdgesByTileType(tile);
                if (value != previousValue) {
                    set(i, j, value);
                }
            }

            hashCode = hashCode * 31 + get(i, j);
            allTilesKnown &= tile != UNKNOWN;
        }
    }

    if (waypoints.empty()) {
        waypoints = world.getWaypoints();
        waypointsHash = width * 31 + height;
        for (auto& waypoint : waypoints) {
            waypointsHash = (waypointsHash * 31 + waypoint[0]) * 31 + waypoint[1];
        }
#ifdef PRINT_MAP
        cout << "{ " << waypointsHash << ", \"" << serializeMap(*this) << "\" }," << endl;
        terminate();
#endif
    }

    if (!allTilesKnown) {
        auto it = PRECALCULATED_MAPS.find(waypointsHash);
        if (it != PRECALCULATED_MAPS.end()) {
            // cout << "precalculated map hash " << it->first << " data " << it->second << endl;
            deserializeMap(*this, it->second);
        }
    }

    Debug::isMap20 |= waypointsHash == 1705052841;
}
