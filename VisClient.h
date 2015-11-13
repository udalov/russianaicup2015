#pragma once

#include "csimplesocket/ActiveSocket.h"
#include "math2d.h"
#include <vector>

using std::string;
using std::vector;

struct VisClient {
    bool valid;
    CActiveSocket socket;

    VisClient(int port);

    void send(const string& message);

    void drawLine(const Point& first, const Point& second);
    void drawPoly(const vector<Point>& points);
};
