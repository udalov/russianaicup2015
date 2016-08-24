#pragma once

#include "math2d.h"
#include "csimplesocket/ActiveSocket.h"

#include <vector>

using std::string;
using std::vector;

struct VisClient {
    bool valid;
    CActiveSocket socket;

    explicit VisClient(int port);

    void send(const string& message);

    void drawLine(const Point& first, const Point& second);
    void drawRect(const Rect& rectangle);
    void drawCircle(const Point& center, double radius);
    void drawText(const Point& point, const string& text);
    void drawTextStatic(const Point& point, const string& text);
};
