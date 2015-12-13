#pragma once

#include "Go.h"
#include "Scorer.h"
#include "State.h"
#include <vector>

using std::vector;

class Track {
public:
    static constexpr double NO_SCORE = -1e42;

    Track() : myMoves(), myScore(NO_SCORE) { }
    explicit Track(const vector<Go>& moves) : myMoves(moves), myScore(NO_SCORE) { }

    double score() const { return myScore; }
    const vector<Go>& moves() const { return myMoves; }
    vector<Go>& moves() { return myMoves; }

    void setScore(double score) {
        this->myScore = score;
    }

    bool operator<(const Track& other) const;

    Track rotate() const;

private:
    vector<Go> myMoves;
    double myScore;
};

void collectTracks(const CarPosition& me, vector<Track>& result);
