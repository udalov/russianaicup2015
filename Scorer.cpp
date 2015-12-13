#include "Scorer.h"
#include "Debug.h"
#include "math2d.h"

#include <algorithm>
#include <iostream>
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

    vis->drawLine(pathSegment[0].p1, pathSegment[0].p2);
    vis->drawLine(pathSegment[0].p1 - 1, pathSegment[0].p2 - 1);
    vis->drawLine(pathSegment[0].p1 + 1, pathSegment[0].p2 + 1);
    vis->drawLine(pathSegment[1].p1, pathSegment[1].p2);
}

double Scorer::scoreTrack(const Track& track) const {
    double result = 0.0;

    auto state = State(startState);
    auto& moves = track.moves();
    for (unsigned long i = 0, size = moves.size(); i < size; i++) {
        state.apply(moves[i]);

        // TODO: constants
        const int firstScoreTurn = 39;
        const int stepScoreTurn = 5;
        if (i >= firstScoreTurn && !((i - firstScoreTurn) % stepScoreTurn)) {
            double stateScore = scoreState(state);
            double damp = ((i - firstScoreTurn + stepScoreTurn) / stepScoreTurn);
            if (damp > 5) damp = 5;
            result += stateScore / damp;
        }
    }

    return result;
}

double Scorer::scoreState(const State& state, bool debug) const {
    auto& me = state.me();

    auto location = me.bumperCenter();

    // TODO: store waypoint index in CarPosition and find shortest path here
    unsigned long startSeg = find(path.begin(), path.end(), startState.me().tile()) - path.begin();
    unsigned long nextSeg = find(path.begin() + startSeg, path.end(), me.tile()) - path.begin();
    if (nextSeg == path.size() || nextSeg - startSeg >= (Debug::isMap11 ? 5 : 7)) return -1e15; // TODO (!)
    nextSeg++;

    auto& next = pathSegment[nextSeg - 1];
    auto cur =
            nextSeg >= 2 ? pathSegment[nextSeg - 2] :
            next + Vec(next.center(), path.front().toPoint()) * 2.0;

    /*
    if (debug) {
        cout << state.original->getTick(); for (auto& tile : path) cout << " " << tile.toString(); cout << endl;
        cout << "  start " << startState.me().tile().toString() << " cur " << me.tile().toString() << " start-seg " << startSeg << " next-seg " << nextSeg
            << " cur " << cur.toString() << " next " << next.toString() << endl;
    }
    */

    auto curHealthDelta = min(me.health - startState.me().health + 0.03, 0.0);

    double result = curHealthDelta * 1e9;

    auto centerLine = Vec(cur.center(), next.center());
    {
        double travelled = (nextSeg - startSeg) + Vec(cur.center(), location).projection(centerLine) / centerLine.length();
        result += travelled;
    }

    result += me.velocity.projection(centerLine) * 0.1;

    result += (me.medicines + me.projectiles + me.nitroCharges + me.oilCanisters + me.pureScore) * 1.0;

    return result;
}
