#pragma once

#include <string>

using std::string;

// TODO (!): add tests
struct Vec3D {
    double x;
    double y;
    double z;

    Vec3D() : x(), y(), z() { }
    Vec3D(double x, double y, double z) : x(x), y(y), z(z) { }
    Vec3D(const Vec3D& other) : x(other.x), y(other.y), z(other.z) { }

    Vec3D *operator=(const Vec3D& other) {
        x = other.x;
        y = other.y;
        return this;
    }

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
