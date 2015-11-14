#pragma once

#define _USE_MATH_DEFINES

#include <cmath>
#include <array>
#include <string>

using std::array;
using std::abs;
using std::string;

const double eps_line_contains = 1e-6;
const double eps_equality = 1e-9;

double mySin(double x);
double myCos(double y);

struct Vec {
    double x;
    double y;

    Vec(double x, double y) : x(x), y(y) { }
    Vec(const Vec& other) : x(other.x), y(other.y) { }
    Vec(double angle) : x(myCos(angle)), y(mySin(angle)) { }

    bool operator==(const Vec& other) const {
        return abs(x - other.x) < eps_equality && abs(y - other.y) < eps_equality;
    }

    double length() const;
    double angle() const;
    Vec normalize() const;
    // Positive angle is counter-clockwise, negative is clockwise
    Vec rotate(double alpha) const;

    Vec project(const Vec& other) const;
    double projection(const Vec& other) const;

    // Negative angle is counter-clockwise, positive is clockwise
    double angleTo(const Vec& other) const;

    Vec operator+(const Vec& other) const;
    Vec operator-(const Vec& other) const;
    Vec operator*(double coeff) const;
    Vec operator-() const;

    void operator+=(const Vec& other);
    void operator-=(const Vec& other);
    void operator*=(double coeff);

    double operator*(const Vec& other) const;
    double operator^(const Vec& other) const;

    string toString() const;
};

struct Point {
    double x;
    double y;

    Point(double x, double y) : x(x), y(y) { }
    Point(const Point& other) : x(other.x), y(other.y) { }

    bool operator==(const Point& other) const {
        return abs(x - other.x) < eps_equality && abs(y - other.y) < eps_equality;
    }

    Point operator+(const Vec& direction) const;
    Point operator-(const Vec& direction) const;

    void operator+=(const Vec& direction);
    void operator-=(const Vec& direction);

    double distanceTo(const Point& other) const;

    string toString() const;
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

double myHypot(double x, double y);
double myAtan2(double y, double x);
double mySin(double x);
double myCos(double y);

// Normalizes the angle to the range [-PI, PI]
double normalizeAngle(double alpha);
