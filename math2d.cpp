#include "math2d.h"

#include <cmath>
#include <sstream>

using namespace std;

Vec::Vec(const Point& p1, const Point& p2) : x(p2.x - p1.x), y(p2.y - p1.y) { }

double Vec::angle() const {
    return myAtan2(y, x);
}

double Vec::length() const {
    return myHypot(x, y);
}

Vec Vec::normalize() const {
    double len = length();
    return abs(len) < 1e-9 ? *this : *this * (1 / len);
}

Vec Vec::project(const Vec& other) const {
    return other.normalize() * projection(other);
}

double Vec::projection(const Vec& other) const {
    return (*this * other) / other.length();
}

double Vec::angleTo(const Vec& other) const {
    return normalizeAngle(angle() - other.angle());
}

Vec Vec::operator*(double coeff) const {
    return Vec(coeff * x, coeff * y);
}

Vec Vec::operator+(const Vec& other) const {
    return Vec(x + other.x, y + other.y);
}

Vec Vec::operator-(const Vec& other) const {
    return Vec(x - other.x, y - other.y);
}

Vec Vec::operator-() const {
    return Vec(-x, -y);
}

void Vec::operator+=(const Vec& other) {
    x += other.x;
    y += other.y;
}

void Vec::operator-=(const Vec& other) {
    x -= other.x;
    y -= other.y;
}

void Vec::operator*=(double coeff) {
    x *= coeff;
    y *= coeff;
}

double Vec::operator*(const Vec& other) const {
    return x * other.x + y * other.y;
}

double Vec::operator^(const Vec& other) const {
    return x * other.y - y * other.x;
}

Point Point::operator+(const Vec& direction) const {
    return Point(x + direction.x, y + direction.y);
}

Point Point::operator-(const Vec& direction) const {
    return Point(x - direction.x, y - direction.y);
}

void Point::operator+=(const Vec& direction) {
    x += direction.x;
    y += direction.y;
}

void Point::operator-=(const Vec& direction) {
    x -= direction.x;
    y -= direction.y;
}

double Point::distanceTo(const Point& other) const {
    return myHypot(x - other.x, y - other.y);
}

Point Point::between(const Point& p1, const Point& p2) {
    return Point((p1.x + p2.x) / 2.0, (p1.y + p2.y) / 2.0);
}

double Line::signedDistanceFrom(const Point& point) const {
    return (a * point.x + b * point.y - c) / myHypot(a, b);
}

double Line::distanceFrom(const Point& point) const {
    return abs(a * point.x + b * point.y - c) / myHypot(a, b);
}

bool Line::contains(const Point& point) const {
    return abs(a * point.x + b * point.y - c) < eps_contains;
}

Point Line::project(const Point& point) const {
    if (contains(point)) return point;
    double d = b * point.x - a * point.y;
    // Ax + By = C
    // Bx - Ay = D
    double e = a * a + b * b;
    double x = (b * d + a * c) / e;
    double y = (b * c - a * d) / e;
    return Point { x, y };
}

Line Line::parallelLine(const Point& point) const {
    return Line(a, b, a * point.x + b * point.y);
}

Vec Line::unitNormalFrom(const Point& point) const {
    double dist = signedDistanceFrom(point);
    double coeff = dist < -eps_normal ? 1 : -1;
    double pseudoLength = myHypot(a, b);
    return Vec(coeff * a / pseudoLength, coeff * b / pseudoLength);
}

double myHypot(double x, double y) {
    // This is faster than hypot
    return mySqrt(x * x + y * y);
}

double myAtan2(double y, double x) {
    // TODO: faster
    return atan2(y, x);
}

double mySin(double x) {
    return sin(x);
    // TODO: fix and enable
    /*
    x /= (M_PI / 2);
    if (x > 0.999999999) x = 2 - x;
    else if (x < -0.999999999) x = -2 - x;
    double x2 = x * x;
    return ((((.00015148419 * x2 - .00467376557) * x2 + .07968967928) * x2 - .64596371106) * x2 + 1.57079631847) * x;
    */
}

double myCos(double y) {
    return mySin(M_PI / 2 - y);
}

double mySqrt(double x) {
    return sqrt(x);
}

double sqr(double x) {
    return x * x;
}

double normalizeAngle(double alpha) {
    while (alpha < -M_PI) alpha += 2 * M_PI;
    while (alpha > M_PI) alpha -= 2 * M_PI;
    return alpha;
}

string Vec::toString() const {
    ostringstream ss;
    ss.precision(3);
    ss << fixed << "(" << x << ", " << y << ")";
    return ss.str();
}

string Point::toString() const {
    ostringstream ss;
    ss.precision(3);
    ss << fixed << "(" << x << ", " << y << ")";
    return ss.str();
}

string Segment::toString() const {
    return string("<") + p1.toString() + " -> " + p2.toString() + ">";
}

string Circle::toString() const {
    ostringstream ss;
    ss.precision(3);
    ss << fixed << "(" << center.x << ", " << center.y << ", " << radius << ")";
    return ss.str();
}

Vec Vec::rotate(double alpha) const {
    auto cos = myCos(alpha);
    auto sin = mySin(alpha);
    return Vec(x * cos - y * sin, x * sin + y * cos);
}

bool withinAABB(double x1, double y1, double x2, double y2, double x, double y) {
    return min(x1, x2) < x + eps_contains && x < max(x1, x2) + eps_contains &&
           min(y1, y2) < y + eps_contains && y < max(y1, y2) + eps_contains;
}

bool withinAABB(const Point& p1, const Point& p2, const Point& p) {
    return withinAABB(p1.x, p1.y, p2.x, p2.y, p.x, p.y);
}

bool segmentContainsPoint(double x1, double y1, double x2, double y2, double x, double y) {
    return withinAABB(x1, y1, x2, y2, x, y) &&
           abs((x - x1) * (y2 - y1) - (y - y1) * (x2 - x1)) < eps_contains;
}

double Rect::distanceFrom(const Point& point) const {
    auto& points = myPoints;
    double result = 1e100;
    for (unsigned long i = 0, size = points.size(); i < size; i++) {
        result = min(result, Segment(points[i], points[i + 1 == size ? 0 : i + 1]).distanceFrom(point));
    }
    return result;
}

Point Segment::center() const {
    return Point((p1.x + p2.x) / 2, (p1.y + p2.y) / 2);
}

double Segment::distanceFrom(const Point& point) const {
    // TODO: optimize
    auto line = Line(p1, p2);
    auto projection = line.project(point);
    if (withinAABB(p1, p2, projection)) return point.distanceTo(projection);
    return min(point.distanceTo(p1), point.distanceTo(p2));
}

bool Segment::contains(const Point& point) const {
    return segmentContainsPoint(p1.x, p1.y, p2.x, p2.y, point.x, point.y);
}

bool Segment::intersects(const Rect& rect, Point& result) const {
    auto& points = rect.points();
    for (unsigned long i = 0, size = points.size(); i < size; i++) {
        if (intersects(points[i], points[i + 1 == size ? 0 : i + 1], result)) return true;
    }
    return false;
}

bool Segment::intersects(const Segment& other, Point& result) const {
    return intersects(other.p1, other.p2, result);
}

bool Segment::intersects(const Point& q1, const Point& q2, Point& result) const {
    // TODO: measure and decide if this check is still needed
    if (min(q1.x, q2.x) > max(p1.x, p2.x) + eps_intersects ||
        min(p1.x, p2.x) > max(q1.x, q2.x) + eps_intersects ||
        min(q1.y, q2.y) > max(p1.y, p2.y) + eps_intersects ||
        min(p1.y, p2.y) > max(q1.y, q2.y) + eps_intersects) return false;

    Vec r(p1, p2), s(q1, q2);
    double rs = r ^ s;
    if (abs(rs) < eps_intersects) return false;

    Vec pq(p1, q1);
    double t = pq ^ s * (1.0 / rs);
    if (t < -eps_intersects || t > 1.0 + eps_intersects) return false;

    double u = pq ^ r * (1.0 / rs);
    if (u < -eps_intersects || u > 1.0 + eps_intersects) return false;

    result = p1 + r * t;
    return true;
    /*
    auto l1 = Line(p1, p2);
    auto l2 = Line(q1, q2);
    return l1.intersects(l2, result) && contains(result) &&
           segmentContainsPoint(q1.x, q1.y, q2.x, q2.y, result.x, result.y);
    */
}

Segment Segment::operator+(const Vec& direction) const {
    return Segment(p1 + direction, p2 + direction);
}

bool Line::intersects(const Line& other, Point& result) const {
    double d = a * other.b - other.a * b;
    if (abs(d) < eps_intersects) return false;
    result.x = (c * other.b - other.c * b) / d;
    result.y = (a * other.c - other.a * c) / d;
    return true;
}

Point Rect::center() const {
    auto& points = myPoints;
    return Point(
            (points[0].x + points[1].x + points[2].x + points[3].x) / 4,
            (points[0].y + points[1].y + points[2].y + points[3].y) / 4
    );
}
