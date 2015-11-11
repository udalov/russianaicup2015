#pragma once

#include "csimplesocket/ActiveSocket.h"

using std::string;

struct VisClient {
    bool valid;
    CActiveSocket socket;

    VisClient(int port);

    void send(const string& message);
};
