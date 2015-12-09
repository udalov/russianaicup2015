#pragma once

#include "Path.h"
#include "State.h"
#include "Tracks.h"
#include "VisClient.h"
#include <vector>

using std::vector;

class Track;

struct Scorer {
    const State& startState;
    vector<Tile> path;

    vector<Segment> pathSegment;

    Scorer(const State& startState, const vector<Tile>& path, VisClient *vis);

    double scoreTrack(const Track& track) const;
    double scoreState(const State& state, bool debug = false) const;
};
