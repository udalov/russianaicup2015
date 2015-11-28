#pragma once

#include "Go.h"
#include "Scorer.h"
#include "State.h"
#include <vector>

using std::vector;

struct Track {
    vector<Go> moves;
    double score;

    Track() : moves(), score(Scorer::NO_SCORE) { }
    Track(const vector<Go>& moves) : moves(moves), score(Scorer::NO_SCORE) { }

    void setScore(double score) {
        this->score = score;
    }

    bool operator<(const Track& other) const;

    Track drop(int ticks) const;
};

void collectTracks(const CarPosition& me, vector<Track>& result);
