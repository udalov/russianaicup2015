#include <cmath>
#include "math2d.h"

using namespace std;

double Vec::angle() const {
    return my_atan2(y, x);
}

double Vec::length() const {
    return my_hypot(x, y);
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
    return normalize_angle(angle() - other.angle());
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

double Vec::operator*(const Vec& other) const {
    return x * other.x + y * other.y;
}

double Vec::operator^(const Vec& other) const {
    return x * other.y - y * other.x;
}

Point Point::shift(const Vec& direction) const {
    return Point(x + direction.x, y + direction.y);
}

bool Line::contains(const Point& point) const {
    return abs(a * point.x + b * point.y - c) < eps_line_contains;
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

double my_hypot(double x, double y) {
    // This is faster than hypot
    return sqrt(x * x + y * y);
}

double my_atan2(double y, double x) {
    // TODO: faster
    return atan2(y, x);
}

double my_sin(double x) {
    x /= (M_PI / 2);
    if (x > 0.999999999) x = 2 - x;
    else if (x < -0.999999999) x = -2 - x;
    double x2 = x * x;
    return ((((.00015148419 * x2 - .00467376557) * x2 + .07968967928) * x2 - .64596371106) * x2 + 1.57079631847) * x;
}

double my_cos(double y) {
    return my_sin(M_PI / 2 - y);
}

double normalize_angle(double alpha) {
    while (alpha < -M_PI) alpha += 2 * M_PI;
    while (alpha > M_PI) alpha -= 2 * M_PI;
    return alpha;
}
