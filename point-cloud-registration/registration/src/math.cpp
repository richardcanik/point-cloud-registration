#include <registration/math.h>

double getLineLength(const Point &a, const Point &b) {
    return sqrt(pow(a.x()-b.x(), 2) + pow(a.y()-b.y(), 2) + pow(a.z()-b.z(), 2));
}

bool checkSameNum(const size_t &i1, const size_t &i2, const size_t &i3, const size_t &i4) {
    return !(i1 == i2 || i1 == i3 || i1 == i4 || i2 == i3 || i2 == i4 || i3 == i4);
}

void sphereParametricEquation(const Point &center, const double &radius, const double &s, const double &t, Point &point) {
    point = center + Point(static_cast<float>(radius * cos(s) * sin(t)), static_cast<float>(radius * sin(s) * sin(t)), static_cast<float>(radius * cos(t)));
}

void lineParametricEquation(const Point &p1, const Point &p2, const float &t, Point &point) {
    point = p1 + (t * (p2 - p1));
}

void circleParametricEquation(const Point &center, const double &radius, const double &t, const Vector3 &a, const Vector3 &b, Point &point) {
    Vector3 v1 = a.normalized();
    Vector3 v2 = a.cross(b).cross(v1).normalized();
    point = center + (radius * sin(t) * v1 + radius * cos(t) * v2);
}

bool isTriangle(const double &a, const double &b, const double &c) {
    return (a + b > c && a + c > b && b + c > a);
}

void rotateVector(const Vector3 &v, const Vector3 &n, const double &angle, Vector3 &out) {
    out = (v * cos(angle)) + (n.cross(v) * sin(angle)) + (n * n.dot(v)) * (1 - cos(angle));
}

void transformPoint(Point &point, const Transform &transform) {
    Vector4 position;
    position = point.head(4);
    position(3) = 1;
    point = (transform * position).head(3);
}
