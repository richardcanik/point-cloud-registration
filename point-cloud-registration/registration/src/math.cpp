#include <registration/math.h>

double getLineLength(const Point &a, const Point &b) {
    return sqrt(pow(a.x-b.x, 2) + pow(a.y-b.y, 2) + pow(a.z-b.z, 2));
}

bool checkSameNum(const size_t &i1, const size_t &i2, const size_t &i3, const size_t &i4) {
    return !(i1 == i2 || i1 == i3 || i1 == i4 || i2 == i3 || i2 == i4 || i3 == i4);
}

void sphereParametricEquation(const Point &center, const double &radius, const double &s, const double &t,
                              Point &point) {
    point = {center.x + static_cast<float>(radius * cos(s) * sin(t)),
             center.y + static_cast<float>(radius * sin(s) * sin(t)),
             center.z + static_cast<float>(radius * cos(t))};
}

void lineParametricEquation(const Point &p1, const Point &p2, const float &t, Point &point) {
    point = {p1.x + (t * (p2.x - p1.x)),
             p1.y + (t * (p2.y - p1.y)),
             p1.z + (t * (p2.z - p1.z))};
}

void circleParametricEquation(const Point &center, const double &radius, const double &t, const Vector &a,
                              const Vector &b, Point &point) {
    Vector v1 = a.cross(b);
    Vector v2 = a.normalized();
    v1 = v1.cross(v2).normalized();
    point = {center.x + static_cast<float>(radius * cos(t) * v1.x() + radius * sin(t) * v2.x()),
             center.y + static_cast<float>(radius * cos(t) * v1.y() + radius * sin(t) * v2.y()),
             center.z + static_cast<float>(radius * cos(t) * v1.z() + radius * sin(t) * v2.z())};
}

bool isTriangle(const double &a, const double &b, const double &c) {
    return (a + b > c && a + c > b && b + c > a);
}

void rotateVector(const Vector &v, const Vector &n, const double &angle, Vector &out) {
    out = (v * cos(angle)) + (n.cross(v) * sin(angle)) + (n * n.dot(v)) * (1 - cos(angle));
}

void transformPoint(Point &point, const Transform &transform) {
    Eigen::Vector4d position, newPosition;
    position(0) = point.x;
    position(1) = point.y;
    position(2) = point.z;
    position(3) = 1;
    newPosition = transform * position;
    point.x = static_cast<float>(newPosition(0));
    point.y = static_cast<float>(newPosition(1));
    point.z = static_cast<float>(newPosition(2));
}
