#include "Tracks.h"
#include "Const.h"
#include "Debug.h"
#include "Go.h"

#include <algorithm>
#include <random>

using namespace std;

#define TURN_LEFT WheelTurnDirection::TURN_LEFT
#define KEEP WheelTurnDirection::KEEP
#define TURN_RIGHT WheelTurnDirection::TURN_RIGHT

bool Track::operator<(const Track& other) const {
    return myScore < other.myScore;
}

Track Track::rotate() const {
    if (myMoves.empty()) return *this;
    auto moves = vector<Go>(myMoves.begin() + 1, myMoves.end());
    moves.push_back(*myMoves.begin());
    return Track(moves);
}

default_random_engine& createRandomEngine() {
    static default_random_engine rng;
    rng.seed(42);
    return rng;
}

vector<Go>& createForwardMoves() {
    static vector<Go> result;
    for (auto& wheelTurn : { TURN_LEFT, KEEP, TURN_RIGHT }) {
        for (bool brake : { false, true }) {
            result.emplace_back(1.0, wheelTurn, brake);
        }
    }
    return result;
}

void collectTracksThreePhases(
        vector<Track>& result, default_random_engine& rng,
        int firstDuration, int secondDuration, int thirdDuration,
        double survivalRate
) {
    static auto& forwardMoves = createForwardMoves();

    uniform_real_distribution<> survival(0, 1);

    vector<Go> moves;
    for (auto& firstMove : forwardMoves) {
        for (int i = 0; i < firstDuration; i++) {
            moves.push_back(firstMove);
        }

        for (auto& secondMove : forwardMoves) {
            for (int j = 0; j < secondDuration; j++) {
                moves.push_back(secondMove);
            }

            for (auto& thirdMove : forwardMoves) {
                if (firstMove.brake && secondMove.brake && thirdMove.brake) {
                    // It seems it's pointless to be braking for that many turns
                    continue;
                }

                for (int k = 0; k < thirdDuration; k++) {
                    moves.push_back(thirdMove);
                }

                if (survival(rng) < survivalRate) {
                    result.emplace_back(moves);
                }
                
                moves.erase(moves.end() - thirdDuration, moves.end());
            }

            moves.erase(moves.end() - secondDuration, moves.end());
        }

        moves.erase(moves.end() - firstDuration, moves.end());
    }
}

void experimentalMutateTracks(vector<Track>& result, default_random_engine& rng) {
    // TODO: experimental stuff

    uniform_int_distribution<int> randTrack(0, result.size() - 1);
    uniform_int_distribution<int> randBool(0, 1);
    for (int i = 0; i < 30; i++) {
        int x = randTrack(rng);
        int y;
        do { y = randTrack(rng); } while (y == x);

        auto& t1 = result[x];
        auto& t2 = result[y];
        vector<Go> moves;
        const unsigned long step = 10;
        for (unsigned long j = 0, size1 = t1.moves().size(), size2 = t2.moves().size(); j < size1 && j < size2; j += step) {
            auto& t = (randBool(rng) & 1) ? t1.moves() : t2.moves();
            for (unsigned long k = 0; k < step && j + k < t.size(); k++) {
                moves.push_back(t[j + k]);
            }
        }
        result.emplace_back(moves);
    }
}

void collectTracks(const CarPosition& me, vector<Track>& result) {
    static auto& rng = createRandomEngine();

    double survivalRate = (Debug::tick % 10) ? 0.1 : 0.2;
    collectTracksThreePhases(result, rng, 5, 25, 50, survivalRate);

    experimentalMutateTracks(result, rng);
}
