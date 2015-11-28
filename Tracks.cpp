#include "Tracks.h"
#include "Const.h"

#include <algorithm>

using namespace std;

bool Track::operator<(const Track& other) const {
    return score < other.score;
}

Track Track::drop(int ticks) const {
    return Track(vector<Go>(moves.begin() + ticks, moves.end()));
}

vector<Go> collectMoves(
        double engineFrom, double engineTo, double engineStep,
        double wheelFrom, double wheelTo, double wheelStep,
        bool brake
) {
    vector<Go> result;
    for (double e = engineFrom; e <= engineTo; e += engineStep) {
        for (double w = wheelFrom; w <= wheelTo; w += wheelStep) {
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

void collectTracks(const CarPosition& me, vector<Track>& result) {
    static const double carEnginePowerChangePerTick = Const::getGame().getCarEnginePowerChangePerTick();
    static const double carWheelTurnChangePerTick = Const::getGame().getCarWheelTurnChangePerTick();

    const int firstDuration = 10;
    const int secondDuration = 25;
    const int thirdDuration = 50;

    // TODO: more turns
    auto firstMoves = collectMoves(1.0, 1.0, 1.0, -1.0, 1.0, 0.25, true);
    for (auto brake : { false, true }) {
        firstMoves.emplace_back(me.enginePower, me.wheelTurn, brake);
    }
    auto secondMovesBase = collectMoves(1.0, 1.0, 1.0, -1.0, 1.0, 1.0, true);
    auto thirdMovesBase = collectMoves(1.0, 1.0, 1.0, -1.0, 1.0, 1.0, true);

    for (unsigned long i = 0; i < firstMoves.size(); i++) {
        auto& go = firstMoves[i];
        bool remove = false;
        if (go.enginePower != -1.0 && go.enginePower != 1.0 &&
            abs(me.enginePower - go.enginePower) > firstDuration * carEnginePowerChangePerTick) {
            remove = true;
        }
        if (go.wheelTurn != -1.0 && go.wheelTurn != 1.0 &&
            abs(me.wheelTurn - go.wheelTurn) > firstDuration * carWheelTurnChangePerTick) {
            remove = true;
        }
        if (remove) {
            firstMoves.erase(firstMoves.begin() + i);
            i--;
        }
    }
    
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

    for (int i = 0; i < 200; i++) {
        int x = rand() % result.size();
        int y;
        do { y = rand() % result.size(); } while (y == x);

        auto& t1 = result[x];
        auto& t2 = result[y];
        vector<Go> moves;
        for (unsigned long j = 0; j < t1.moves.size(); j += 10) {
            auto& t = (rand() & 1) ? t1 : t2;
            for (int k = 0; k < 10 && j + k < t.moves.size(); k++) {
                moves.push_back(t.moves[j + k]);
            }
        }
        result.emplace_back(moves);
    }
}
