#include "Collider.h"

#include <algorithm>
#include <iostream>

using namespace std;

bool collideRectAndSegment(const Rect& rect, const Segment& segment, CollisionInfo& result) {
    auto& points = rect.points;
    auto intersections = 0;
    Point intersectionSum;
    for (unsigned long i = 0, size = points.size(); i < size; i++) {
        auto side = Segment(points[i], points[i + 1 == size ? 0 : i + 1]);
        Point intersection;
        if (side.intersects(segment, intersection)) {
            intersections++;
            intersectionSum.x += intersection.x;
            intersectionSum.y += intersection.y;
        }
    }
    if (intersections != 2) return false;

    auto line = Line(segment.p1, segment.p2);

    const Point *minDistPoint = &points[0];
    double minDist = line.signedDistanceFrom(*minDistPoint);
    const Point *maxDistPoint = minDistPoint;
    double maxDist = minDist;

    for (unsigned long i = 1, size = points.size(); i < size; i++) {
        auto& point = points[i];
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

    result.point = Point(intersectionSum.x / 2, intersectionSum.y / 2);

    if (line.signedDistanceFrom(rect.center()) > 0.0) {
        result.normal = line.parallelLine(*minDistPoint).unitNormalFrom(*maxDistPoint);
        result.depth = abs(minDist);
    } else {
        result.normal = line.parallelLine(*maxDistPoint).unitNormalFrom(*minDistPoint);
        result.depth = maxDist;
    }

    return true;
}

bool collectCircleAndSegment(const Segment& segment, const Circle& circle, CollisionInfo& result) {
    double distance = segment.distanceFrom(circle.center);
    if (distance > circle.radius || distance < 1e-9) return false;

    // TODO: optimize
    auto line = Line(segment.p1, segment.p2);
    auto projection = line.project(circle.center);
    if (segment.contains(projection)) {
        result.point = projection;
        result.normal = Vec(projection, circle.center).normalize();
        result.depth = circle.radius - distance;
        return true;
    }

    double dist1 = circle.center.distanceTo(segment.p1);
    double dist2 = circle.center.distanceTo(segment.p2);
    double minDist;
    if (dist1 < dist2) {
        result.point = segment.p1;
        minDist = dist1;
    } else {
        result.point = segment.p2;
        minDist = dist2;
    }

    result.normal = Vec(result.point, circle.center).normalize();
    result.depth = circle.radius - minDist;

    return true;
}

bool collideCircleAndRect(const Rect& rect, const Circle& circle, CollisionInfo& result) {
    auto& points = rect.points;

    bool collision = false;
    double maxDepth = -1e100;
    CollisionInfo local;
    for (unsigned long i = 0, size = points.size(); i < size; i++) {
        auto side = Segment(points[i], points[i + 1 == size ? 0 : i + 1]);
        if (collectCircleAndSegment(side, circle, local) && local.depth > maxDepth) {
            collision = true;
            result = local;
            maxDepth = local.depth;
        }
    }

    return collision;
}
