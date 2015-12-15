#include "math3d.h"
#include "math2d.h"

#include <sstream>

using namespace std;

double Vec3D::normSquare() const {
    return x * x + y * y + z * z;
}

double Vec3D::norm() const {
    return mySqrt(normSquare());
}

Vec3D Vec3D::normalize() const {
    double s = norm();
    return Vec3D(x / s, y / s, z / s);
}

Vec3D Vec3D::operator*(double coeff) const {
    return Vec3D(x * coeff, y * coeff, z * coeff);
}

Vec3D Vec3D::operator-() const {
    return Vec3D(-x, -y, -z);
}

Vec3D Vec3D::operator+(const Vec3D& other) const {
    return Vec3D(x + other.x, y + other.y, z + other.z);
}

Vec3D Vec3D::operator-(const Vec3D& other) const {
    return Vec3D(x - other.x, y - other.y, z - other.z);
}

Vec3D Vec3D::operator^(const Vec3D& other) const {
    return Vec3D(
            y * other.z - z * other.y,
            z * other.x - x * other.z,
            x * other.y - y * other.x
    );
}

double Vec3D::operator*(const Vec3D& other) const {
    return x * other.x + y * other.y + z * other.z;
}

string Vec3D::toString() const {
    ostringstream ss;
    ss.precision(3);
    ss << fixed << "(" << x << ", " << y << ", " << z << ")";
    return ss.str();
}

ostream& operator<<(ostream& out, const Vec3D& vec) {
    out << vec.toString();
    return out;
}
