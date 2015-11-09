#include "Runner.h"

#include <vector>

#include "MyStrategy.h"

using namespace model;
using namespace std;

Runner::Runner(const char* host, const char* port, const char* token)
        : remoteProcessClient(host, atoi(port)), token(token) {
}

void Runner::run() {
    remoteProcessClient.writeTokenMessage(token);
    int teamSize = remoteProcessClient.readTeamSizeMessage();
    remoteProcessClient.writeProtocolVersionMessage();
    Game game = remoteProcessClient.readGameContextMessage();

    vector<Strategy*> strategies;

    for (int strategyIndex = 0; strategyIndex < teamSize; ++strategyIndex) {
        Strategy* strategy = new MyStrategy;
        strategies.push_back(strategy);
    }

    PlayerContext* playerContext;

    while ((playerContext = remoteProcessClient.readPlayerContextMessage()) != NULL) {
        vector<Car> playerCars = playerContext->getCars();
        if ((int) playerCars.size() != teamSize) {
            break;
        }

        vector<Move> moves;

        for (int carIndex = 0; carIndex < teamSize; ++carIndex) {
            Car playerCar = playerCars[carIndex];

            Move move;
            strategies[playerCar.getTeammateIndex()]->move(playerCar, playerContext->getWorld(), game, move);
            moves.push_back(move);
        }

        remoteProcessClient.writeMovesMessage(moves);

        delete playerContext;
    }

    for (int strategyIndex = 0; strategyIndex < teamSize; ++strategyIndex) {
        delete strategies[strategyIndex];
    }
}
