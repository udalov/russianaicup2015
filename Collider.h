#pragma once

#include "math2d.h"

struct CollisionInfo {
    Point point;
    Vec normal;
    double depth;

    CollisionInfo() : point(), normal(), depth() { }
    CollisionInfo(const Point& point, const Vec& normal, double depth) :
        point(point), normal(normal), depth(depth) { }
};

bool collideRectAndSegment(const Rect& rect, const Segment& segment, CollisionInfo& result);

bool collideRectAndCircle(const Rect& rect, const Circle& circle, CollisionInfo& result);
