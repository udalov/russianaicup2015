#pragma once

#include "Path.h"
#include "State.h"
#include "VisClient.h"
#include <vector>

using std::vector;

struct Scorer {
    static constexpr double NO_SCORE = -1e42;

    const State& startState;
    const vector<Tile>& path;

    vector<Segment> pathSegment;

    Scorer(const State& startState, const vector<Tile>& path, VisClient *vis);

    double score(const State& state, bool debug = false) const;
};
