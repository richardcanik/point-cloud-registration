#ifndef SRC_MATH_H
#define SRC_MATH_H

#include <Eigen/Eigen>
#include <iostream>

typedef Eigen::Vector3f Vector3;
typedef Eigen::Vector4f Vector4;
typedef Eigen::Vector3f Point;
typedef Eigen::Matrix4f Transform;

struct VoxelCoordinate {
    long x = 0;
    long y = 0;
    long z = 0;
};

struct Condition {
    const Point *point = nullptr;
    const double *descriptor = nullptr;
};

enum POINT {
    P2,
    P3,
    P4
};

enum QUADRANT {
    ONE,
    TWO,
    THREE,
    FOUR
};

enum INTERSECTION_STATUS {
    NONE,
    TOUCH_POINT,
    MORE
};

double getLineLength(const Point &a, const Point &b);
void getLineLength(const Point &a, const Point &b, double &out);
int sumToZero(const int &start);
bool checkSameNum(const size_t &i1, const size_t &i2, const size_t &i3, const size_t &i4);
void sphereParametricEquation(const Point &center, const double &radius, const double &s, const double &t, Point &point);
void lineParametricEquation(const Point &p1, const Point &p2, const float &t, Point &point);
void circleParametricEquation(const Point &center, const double &radius, const double &t, const Vector3 &a, const Vector3 &b, Point &point);
bool isTriangle(const double &a, const double &b, const double &c);
void transformPoint(Point &point, const Transform &transform);
void rotateVector(const Vector3 &v, const Vector3 &n, const double &angle, Vector3 &out);
void twoSpheresIntersection(const Point &center1, const double &radius1, const Point &center2, const double &radius2,
                            Point &center, double &radius, Vector3 &v1, Vector3 &v2, INTERSECTION_STATUS &status);
void circleSphereIntersection(const Point &centerCircle, const double &radiusCircle, const Vector3 &v1, const Vector3 &v2,
                              const Point &centerSphere, const double &radiusSphere,
                              std::vector<Point> &points, INTERSECTION_STATUS &status);

#endif //SRC_MATH_H
