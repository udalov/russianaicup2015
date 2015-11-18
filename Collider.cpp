#include "Collider.h"

#include <algorithm>
#include <iostream>

using namespace std;

bool collideRectAndSegment(const Rect& rect, const Segment& segment, CollisionInfo& result) {
    // TODO: optimize
    auto& rectPoints = rect.points;
    vector<Point> intersections;
    for (unsigned long i = 0, size = rectPoints.size(); i < size; i++) {
        auto side = Segment(rectPoints[i], rectPoints[i + 1 == size ? 0 : i + 1]);
        Point intersection;
        if (side.intersects(segment, intersection)) {
            intersections.push_back(intersection);
        }
    }
    if (intersections.size() != 2) return false;

    auto line = Line(segment.p1, segment.p2);

    const Point *minDistPoint = &rectPoints[0];
    double minDist = line.signedDistanceFrom(*minDistPoint);
    const Point *maxDistPoint = minDistPoint;
    double maxDist = minDist;

    for (unsigned long i = 1, size = rectPoints.size(); i < size; i++) {
        auto& point = rectPoints[i];
        auto curDist = line.signedDistanceFrom(point);
        if (curDist < minDist) {
            minDist = curDist;
            minDistPoint = &point;
        }
        if (curDist > maxDist) {
            maxDist = curDist;
            maxDistPoint = &point;
        }
    }

    if ((minDist < 0.0 && maxDist < 0.0) || (minDist > 0.0 && maxDist > 0.0)) return false;

    result.point = Point(
            (intersections[0].x + intersections[1].x) / 2,
            (intersections[0].y + intersections[1].y) / 2
    );

    if (line.signedDistanceFrom(rect.center()) > 0.0) {
        result.normal = line.parallelLine(*minDistPoint).unitNormalFrom(*maxDistPoint);
        result.depth = abs(minDist);
    } else {
        result.normal = line.parallelLine(*maxDistPoint).unitNormalFrom(*minDistPoint);
        result.depth = maxDist;
    }

    return true;
}

bool collideRectAndCircle(const Rect& rect, const Circle& circle, CollisionInfo& result) {
    // TODO
    return false;
}
