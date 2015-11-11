#pragma once

#include <array>
#include <cmath>

using std::array;
using std::abs;

const double eps_line_contains = 1e-6;
const double eps_equality = 1e-9;

double my_sin(double x);
double my_cos(double y);

struct Vec {
    double x;
    double y;

    Vec(double x, double y) : x(x), y(y) { }
    Vec(const Vec& other) : x(other.x), y(other.y) { }
    Vec(double angle) : x(my_cos(angle)), y(my_sin(angle)) { }

    bool operator==(const Vec& other) const {
        return abs(x - other.x) < eps_equality && abs(y - other.y) < eps_equality;
    }

    double length() const;
    double angle() const;
    Vec normalize() const;

    Vec project(const Vec& other) const;
    double projection(const Vec& other) const;

    // Negative angle is counter-clockwise, positive is clockwise
    double angleTo(const Vec& other) const;

    Vec operator+(const Vec& other) const;
    Vec operator-(const Vec& other) const;
    Vec operator*(double coeff) const;
    Vec operator-() const;

    double operator*(const Vec& other) const;
    double operator^(const Vec& other) const;
};

struct Point {
    double x;
    double y;

    Point(double x, double y) : x(x), y(y) { }
    Point(const Point& other) : x(other.x), y(other.y) { }

    bool operator==(const Point& other) const {
        return abs(x - other.x) < eps_equality && abs(y - other.y) < eps_equality;
    }

    Point shift(const Vec& direction) const;
};

struct Line {
    // Ax + By = C
    double a;
    double b;
    double c;

    Line(const Point& p1, const Point& p2) :
            a(p1.y - p2.y),
            b(p2.x - p1.x),
            c(a * p1.x + b * p1.y) { }

    bool contains(const Point& point) const;

    Point project(const Point& point) const;
};

double my_hypot(double x, double y);
double my_atan2(double y, double x);
double my_sin(double x);
double my_cos(double y);

// Normalizes the angle to the range [-PI, PI]
double normalize_angle(double alpha);
