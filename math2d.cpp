#include "math2d.h"

#include <cmath>
#include <sstream>

using namespace std;

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

double myHypot(double x, double y) {
    // This is faster than hypot
    return sqrt(x * x + y * y);
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

double normalizeAngle(double alpha) {
    while (alpha < -M_PI) alpha += 2 * M_PI;
    while (alpha > M_PI) alpha -= 2 * M_PI;
    return alpha;
}

string Vec::toString() const {
    ostringstream ss;
    ss.precision(8);
    ss << fixed << "(" << x << ", " << y << ")";
    return ss.str();
}

string Point::toString() const {
    ostringstream ss;
    ss.precision(8);
    ss << fixed << "(" << x << ", " << y << ")";
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

bool segmentContainsPoint(double x1, double y1, double x2, double y2, double x, double y) {
    return withinAABB(x1, y1, x2, y2, x, y) &&
           abs((x - x1) * (y2 - y1) - (y - y1) * (x2 - x1)) < eps_contains;
}

double Rect::distanceFrom(const Point& point) const {
    double result = 1e100;
    for (unsigned long i = 0, size = points.size(); i < size; i++) {
        result = min(result, Segment(points[i], points[i + 1 == size ? 0 : i + 1]).distanceFrom(point));
    }
    return result;
}

double Segment::distanceFrom(const Point& point) const {
    auto line = Line(p1, p2);
    auto projection = line.project(point);
    if (withinAABB(p1.x, p1.y, p2.x, p2.y, projection.x, projection.y)) return point.distanceTo(projection);
    return min(point.distanceTo(p1), point.distanceTo(p2));
}

bool Segment::contains(const Point& point) const {
    return segmentContainsPoint(p1.x, p1.y, p2.x, p2.y, point.x, point.y);
}

bool Segment::intersects(const Rect& rect) const {
    auto& points = rect.points;
    for (unsigned long i = 0, size = points.size(); i < size; i++) {
        if (intersects(points[i], points[i + 1 == size ? 0 : i + 1])) return true;
    }
    return false;
}

bool Segment::intersects(const Segment& other) const {
    return intersects(other.p1, other.p2);
}

bool Segment::intersects(const Point& q1, const Point& q2) const {
    if (min(q1.x, q2.x) > max(p1.x, p2.x) + eps_intersects ||
        min(p1.x, p2.x) > max(q1.x, q2.x) + eps_intersects ||
        min(q1.y, q2.y) > max(p1.y, p2.y) + eps_intersects ||
        min(p1.y, p2.y) > max(q1.y, q2.y) + eps_intersects) return false;
    // TODO: optimize
    auto l1 = Line(p1, p2);
    auto l2 = Line(q1, q2);
    Point p;
    return l1.intersect(l2, p) && contains(p) && segmentContainsPoint(q1.x, q1.y, q2.x, q2.y, p.x, p.y);
}

bool Line::intersect(const Line& other, Point& result) const {
    double d = a * other.b - other.a * b;
    if (abs(d) < eps_intersects) return false;
    result.x = (c * other.b - other.c * b) / d;
    result.y = (a * other.c - other.a * c) / d;
    return true;
}
