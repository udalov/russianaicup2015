#include "Scorer.h"
#include "math2d.h"

#include <algorithm>
#include <cmath>

using namespace std;

Segment segmentBetweenTiles(const Tile& tile1, const Tile& tile2) {
    static const double angle = M_PI / 6;
    static const double coeff = 0.5 / myCos(angle);
    auto p1 = tile1.toPoint();
    auto p2 = tile2.toPoint();
    auto v = Vec(p1, p2) * coeff;
    return Segment(p1 + v.rotate(angle), p1 + v.rotate(-angle));
}

Scorer::Scorer(const State& startState, const vector<Tile>& path, VisClient *vis) : startState(startState), path(path) {
    for (unsigned long i = 0, size = path.size(); i < size - 1; i++) {
        pathSegment.push_back(segmentBetweenTiles(path[i], path[i + 1]));
    }

    vis->drawLine(pathSegment.front().p1, pathSegment.front().p2);
}

bool isFacingTowards(const CarPosition& car, const Segment& segment) {
    auto& p = car.location;
    return (Vec(p, segment.p1) ^ car.velocity) < 0 && (car.velocity ^ Vec(p, segment.p2)) < 0;
}

double Scorer::score(const State& state, bool debug) const {
    auto& me = state.me();

    auto location = me.bumperCenter();

    unsigned long pathIndex = find(path.begin(), path.end(), startState.me().tile()) - path.begin() + 1;
    if (me.tile() == path[pathIndex] && pathIndex + 1 < path.size()) pathIndex++;

    /*
    if (state.original->getTick() >= 700) {
        cout << state.original->getTick();
        for (auto& tile : path) cout << " " << tile.toString();
        cout << endl << "  start " << startState.me().tile().toString() << " cur " << me.tile().toString() << " " << pathIndex << endl;
    }
    */

    auto& next = pathSegment[pathIndex - 1];
    auto cur =
            pathIndex >= 2 ? pathSegment[pathIndex - 2] :
            next + Vec(next.center(), path.front().toPoint()) * 2.0;
    // auto& nextNext = pathSegment[pathIndex];
    // auto& nextNextNext = pathSegment[pathIndex + 1];

    auto curHealthDelta = min(me.health - startState.me().health + 0.03, 0.0);

    double result = curHealthDelta * 1e9;

    auto centerLine = Vec(cur.center(), next.center());
    {
        double travelled = pathIndex + Vec(cur.center(), location).projection(centerLine) / centerLine.length();
        result += travelled;
    }

    /*
    {
        auto point = next.center();
        auto nextNextCenter = nextNext.center();
        auto d1 = next.p1.distanceTo(nextNextCenter);
        auto d2 = next.p2.distanceTo(nextNextCenter);
        if (abs(d1 - d2) > 1e-2) {
            auto curCenter = cur.center();
            if (abs(next.p1.distanceTo(curCenter) - next.p2.distanceTo(curCenter)) > 1e-2) {
                point = d1 < d2 ? next.p1 : next.p2;
            } else {
                point = d1 > d2 ? next.p1 : next.p2;
            }
        }

        result += pathIndex * 5e6 - point.distanceTo(me.location) * 1e3;
    }
    */

    /*
    auto dir = me.direction();
    if (!isFacingTowards(me, next)) {
        auto minAngle = min(
                abs(dir.angleTo(Vec(location, next.p1))),
                abs(dir.angleTo(Vec(location, next.p2)))
        );
        result -= minAngle * 500.0;
    }
    */

    /*
    auto nextTurnAngle = abs(dir.angleTo(Vec(next.center(), nextNextNext.center())));
    result -= nextTurnAngle * 1400.0;
    */

    result += (me.velocity.projection(centerLine) / centerLine.length()) * 0.1;

    /*
    Point center;
    bool turn = false;
    if (cur.p1.y == cur.p2.y && next.p1.x == next.p2.x) {
        turn = true;
        center = Point(next.p1.x, cur.p1.y);
    } else if (cur.p1.x == cur.p2.x && next.p1.y == next.p2.y) {
        turn = true;
        center = Point(cur.p1.x, next.p1.y);
    }

    double turnAngle;
    if (turn) {
        auto angle = Vec(center, next.center()).angleTo(Vec(center, cur.center()));
        // if (debug) cout << "  angle " << angle;
        auto v = Vec(center, location).rotate(angle);
        turnAngle = abs(dir.angleTo(v));
    } else {
        turnAngle = abs(dir.angleTo(Vec(cur.center(), next.center())));
    }

    // if (debug) cout << "  turn-angle " << turnAngle << endl;

    result -= turnAngle * 0.1;
    */

    /*
    unsigned long j = pathIndex - 1;
    while (j + 1 < pathSegment.size() &&
           abs(pathSegment[j].p1.distanceTo(pathSegment[j + 1].center()) - pathSegment[j].p2.distanceTo(pathSegment[j + 1].center())) < 1e-2) {
        j++;
    }
    Point far = pathSegment[j].p1.distanceTo(pathSegment[j + 1].center()) > pathSegment[j].p2.distanceTo(pathSegment[j + 1].center())
        ? pathSegment[j].p1 : pathSegment[j].p2;
    Point g = cur.p1.distanceTo(far) < cur.p2.distanceTo(far) ? cur.p1 : cur.p2;
    auto dist = Line(far, g).distanceFrom(me.location);

    result -= dist * 0.5;
    */

    /*
    double angleDiff = Vec(cur.center(), next.center()).angleTo(me.direction());
    result -= abs(angleDiff) * 1000.0;
    */

    return result;
}
