#pragma once

#include "math2d.h"

struct CollisionInfo {
    Point point;
    Vec normal;
    double depth;

    CollisionInfo() : point(), normal(), depth() { }
    CollisionInfo(const Point& point, const Vec& normal, double depth) :
            point(point), normal(normal), depth(depth) { }
    CollisionInfo(const CollisionInfo& other) :
            point(other.point), normal(other.normal), depth(other.depth) { }

    CollisionInfo *operator=(const CollisionInfo& other) {
        point = other.point;
        normal = other.normal;
        depth = other.depth;
        return this;
    }
};

bool collideRectAndSegment(const Rect& rect, const Segment& segment, CollisionInfo& result);

bool collideCircleAndRect(const Rect& rect, const Circle& circle, CollisionInfo& result);
