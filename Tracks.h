#pragma once

#include "Go.h"
#include "Scorer.h"
#include "State.h"
#include <vector>

using std::vector;

class Track {
public:
    Track() : myMoves(), myScore(Scorer::NO_SCORE) { }
    Track(const vector<Go>& moves) : myMoves(moves), myScore(Scorer::NO_SCORE) { }

    double score() const { return myScore; }
    const vector<Go>& moves() const { return myMoves; }

    void setScore(double score) {
        this->myScore = score;
    }

    bool operator<(const Track& other) const;

    Track drop(unsigned long ticks) const;

private:
    vector<Go> myMoves;
    double myScore;
};

void collectTracks(const CarPosition& me, vector<Track>& result);
