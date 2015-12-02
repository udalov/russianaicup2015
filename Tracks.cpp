#include "Tracks.h"
#include "Const.h"
#include "Go.h"

#include <algorithm>
#include <random>

using namespace std;

#define TURN_LEFT WheelTurnDirection::TURN_LEFT
#define KEEP WheelTurnDirection::KEEP
#define TURN_RIGHT WheelTurnDirection::TURN_RIGHT

const double SURVIVAL_RATE = 0.15;

bool Track::operator<(const Track& other) const {
    return myScore < other.myScore;
}

Track Track::rotate() const {
    if (myMoves.empty()) return *this;
    auto moves = vector<Go>(myMoves.begin() + 1, myMoves.end());
    moves.push_back(*myMoves.begin());
    return Track(moves);
}

default_random_engine createRandomEngine() {
    default_random_engine rng;
    rng.seed(42);
    return rng;
}

void collectTracks(const CarPosition& me, vector<Track>& result) {
    static auto rng = createRandomEngine();

    const int firstDuration = 5;
    const int secondDuration = 25;
    const int thirdDuration = 50;

    vector<Go> forwardMoves;
    for (auto& wheelTurn : { TURN_LEFT, KEEP, TURN_RIGHT }) {
        for (bool brake : { false, true }) {
            forwardMoves.emplace_back(1.0, wheelTurn, brake);
        }
    }

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

                if (survival(rng) < SURVIVAL_RATE) {
                    result.emplace_back(moves);
                }
                
                moves.erase(moves.end() - thirdDuration, moves.end());
            }

            moves.erase(moves.end() - secondDuration, moves.end());
        }

        moves.erase(moves.end() - firstDuration, moves.end());
    }

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
        for (unsigned long j = 0, size1 = t1.moves().size(), size2 = t2.moves().size(); j < size1 && j < size2; j += 10) {
            auto& t = (randBool(rng) & 1) ? t1.moves() : t2.moves();
            for (int k = 0; k < 10 && j + k < t.size(); k++) {
                moves.push_back(t[j + k]);
            }
        }
        result.emplace_back(moves);
    }
}
