#pragma once

#include <ostream>
#include <string>

using std::ostream;
using std::string;

// TODO (!): add tests
struct Vec3D {
    double x;
    double y;
    double z;

    Vec3D() : x(), y(), z() { }
    explicit Vec3D(double x, double y, double z) : x(x), y(y), z(z) { }

    double normSquare() const;
    double norm() const;

    Vec3D normalize() const;

    Vec3D operator*(double coeff) const;
    Vec3D operator-() const;

    Vec3D operator+(const Vec3D& other) const;
    Vec3D operator-(const Vec3D& other) const;
    Vec3D operator^(const Vec3D& other) const;

    double operator*(const Vec3D& other) const;

    string toString() const;
};

ostream& operator<<(ostream& out, const Vec3D& vec);
