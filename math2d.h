#pragma once

#ifdef __STRICT_ANSI__
#undef __STRICT_ANSI__
#endif

#include <cmath>
#include <iostream>
#include <string>
#include <vector>

using std::abs;
using std::ostream;
using std::string;
using std::vector;

const double eps_contains = 1e-6;
const double eps_equality = 1e-9;
const double eps_intersects = 1e-9;
const double eps_normal = 1e-9;

double mySin(double x);
double myCos(double y);

struct Point;

struct Vec {
    double x;
    double y;

    Vec() : x(), y() { }
    explicit Vec(double x, double y) : x(x), y(y) { }
    explicit Vec(double angle) : x(myCos(angle)), y(mySin(angle)) { }
    explicit Vec(const Point& p1, const Point& p2);

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

    Point() : x(), y() { }
    explicit Point(double x, double y) : x(x), y(y) { }

    bool operator==(const Point& other) const {
        return abs(x - other.x) < eps_equality && abs(y - other.y) < eps_equality;
    }

    Point operator+(const Vec& direction) const;
    Point operator-(const Vec& direction) const;

    void operator+=(const Vec& direction);
    void operator-=(const Vec& direction);

    double distanceTo(const Point& other) const;

    string toString() const;

    static Point between(const Point& p1, const Point& p2);
};

struct Line {
    // Ax + By = C
    double a;
    double b;
    double c;

    explicit Line(double a, double b, double c) : a(a), b(b), c(c) { }
    explicit Line(const Point& p1, const Point& p2) :
            a(p1.y - p2.y),
            b(p2.x - p1.x),
            c(a * p1.x + b * p1.y) { }

    double distanceFrom(const Point& point) const;
    double signedDistanceFrom(const Point& point) const;

    // TODO: test
    Line parallelLine(const Point& point) const;
    // TODO: test
    Vec unitNormalFrom(const Point& point) const;

    bool contains(const Point& point) const;

    Point project(const Point& point) const;

    bool intersects(const Line& other, Point& result) const;
};

class Rect {
public:
    explicit Rect(const vector<Point>& points) : myPoints(points) { }

    double distanceFrom(const Point& point) const;

    Point center() const;

    const vector<Point>& points() const { return myPoints; }
    vector<Point>& points() { return myPoints; }

private:
    vector<Point> myPoints;
};

struct Segment {
    Point p1;
    Point p2;

    explicit Segment(double x1, double y1, double x2, double y2) : p1(x1, y1), p2(x2, y2) { }
    explicit Segment(const Point& p1, const Point& p2) : p1(p1), p2(p2) { }

    Point center() const;

    // TODO (!): test
    double distanceFrom(const Point& point) const;

    bool contains(const Point& point) const;

    bool intersects(const Point& q1, const Point& q2, Point& result) const;
    bool intersects(const Rect& rect, Point& result) const;
    bool intersects(const Segment& other, Point& result) const;

    Segment operator+(const Vec& direction) const;

    string toString() const;
};

struct Circle {
    Point center;
    double radius;

    explicit Circle(const Point& center, double radius) : center(center), radius(radius) { }

    string toString() const;
};

ostream& operator<<(ostream& out, const Vec& vec);
ostream& operator<<(ostream& out, const Point& point);
ostream& operator<<(ostream& out, const Segment& segment);
ostream& operator<<(ostream& out, const Circle& circle);

double myHypot(double x, double y);
double myAtan2(double y, double x);
double mySin(double x);
double myCos(double y);
double mySqrt(double x);
double sqr(double x);

// Normalizes the angle to the range [-PI, PI]
double normalizeAngle(double alpha);

bool withinAABB(double x1, double y1, double x2, double y2, double x, double y);
bool withinAABB(const Point& p1, const Point& p2, const Point& p);
