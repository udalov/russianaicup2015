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

bool Segment::contains(const Point& point) const {
    return min(p1.x, p2.x) <= point.x + eps_contains && point.x <= max(p1.x, p2.x) + eps_contains &&
           min(p1.y, p2.y) <= point.y + eps_contains && point.y <= max(p1.y, p2.y) + eps_contains &&
           abs((point.x - p1.x) * (p2.y - p1.y) - (point.y - p1.y) * (p2.x - p1.x)) < eps_contains;
}

bool Segment::intersects(const Rectangle& rect) const {
    for (auto it = rect.points.begin(); it != rect.points.end(); ++it) {
        if (intersects(Segment(*it, it + 1 == rect.points.end() ? rect.points.front() : *(it + 1)))) return true;
    }
    return false;
}

bool Segment::intersects(const Segment& other) const {
    // TODO: optimize
    auto l1 = Line(p1, p2);
    auto l2 = Line(other.p1, other.p2);
    Point p;
    return l1.intersect(l2, p) && contains(p) && other.contains(p);
}

bool Line::intersect(const Line& other, Point& result) const {
    double d = a * other.b - other.a * b;
    if (abs(d) < eps_intersects) return false;
    result.x = (c * other.b - other.c * b) / d;
    result.y = (a * other.c - other.a * c) / d;
    return true;
}
