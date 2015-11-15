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
    void drawRect(const Rectangle& rectangle);
    void drawPoly(const vector<Point>& points);
    void drawCircle(const Point& center, double radius);
};
