#include "Tracks.h"
#include "Const.h"
#include "Go.h"

#include <algorithm>
#include <random>

using namespace std;

using WheelTurnDirection::TURN_LEFT;
using WheelTurnDirection::KEEP;
using WheelTurnDirection::TURN_RIGHT;

bool Track::operator<(const Track& other) const {
    return score < other.score;
}

Track Track::drop(unsigned long ticks) const {
    return Track(vector<Go>(moves.begin() + min(ticks, moves.size()), moves.end()));
}

vector<Go> collectMoves(double engineFrom, double engineTo, double engineStep, bool brake) {
    vector<Go> result;
    for (double e = engineFrom; e <= engineTo; e += engineStep) {
        for (auto& w : { TURN_LEFT, KEEP, TURN_RIGHT }) {
            for (int b = 0; b <= brake; b++) {
                result.emplace_back(e, w, (bool) b);
            }
        }
    }
    return result;
}

template <typename T> vector<T> addIfNeeded(const vector<T>& v, const T& element) {
    if (find(v.begin(), v.end(), element) == v.end()) {
        auto result = v;
        result.push_back(element);
        return result;
    }
    return v;
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

    auto firstMoves = collectMoves(1.0, 1.0, 1.0, true);
    if (me.enginePower != 1.0) {
        for (auto& wheel : { TURN_LEFT, KEEP, TURN_RIGHT }) {
            for (auto brake : { false, true }) {
                firstMoves.emplace_back(me.enginePower, wheel, brake);
            }
        }
    }
    auto secondMovesBase = collectMoves(1.0, 1.0, 1.0, true);
    auto thirdMovesBase = collectMoves(1.0, 1.0, 1.0, true);

    for (auto& firstMove : firstMoves) {
        vector<Go> moves;
        for (int i = 0; i < firstDuration; i++) {
            moves.push_back(firstMove);
        }

        auto secondMoves = addIfNeeded(secondMovesBase, firstMove);
        for (auto& secondMove : secondMoves) {
            for (int j = 0; j < secondDuration; j++) {
                moves.push_back(secondMove);
            }

            auto thirdMoves = addIfNeeded(addIfNeeded(thirdMovesBase, firstMove), secondMove);
            for (auto& thirdMove : thirdMoves) {
                for (int k = 0; k < thirdDuration; k++) {
                    moves.push_back(thirdMove);
                }
                result.emplace_back(moves);
                moves.erase(moves.end() - thirdDuration, moves.end());
            }

            moves.erase(moves.end() - secondDuration, moves.end());
        }

        moves.erase(moves.end() - firstDuration, moves.end());
    }

    // TODO: experimental stuff

    uniform_int_distribution<int> randTrack(0, result.size() - 1);
    uniform_int_distribution<int> randBool(0, 1);
    for (int i = 0; i < 200; i++) {
        int x = randTrack(rng);
        int y;
        do { y = randTrack(rng); } while (y == x);

        auto& t1 = result[x];
        auto& t2 = result[y];
        vector<Go> moves;
        for (unsigned long j = 0; j < t1.moves.size(); j += 10) {
            auto& t = (randBool(rng) & 1) ? t1 : t2;
            for (int k = 0; k < 10 && j + k < t.moves.size(); k++) {
                moves.push_back(t.moves[j + k]);
            }
        }
        result.emplace_back(moves);
    }
}