#pragma once

#include "csimplesocket/ActiveSocket.h"

using std::string;

struct VisClient {
    bool valid;
    CActiveSocket socket;

    VisClient(int port);

    void send(const string& message);

    void drawLine(double x1, double y1, double x2, double y2);
};
