#ifndef SRC_MATH_H
#define SRC_MATH_H

#include <Eigen/Eigen>
#include <iostream>

typedef Eigen::Vector3d Vector3;
typedef Eigen::Vector4d Vector4;
typedef Eigen::Matrix3d Matrix3;
typedef Eigen::Matrix4d Matrix4;
typedef Eigen::Vector3d Point;
typedef Eigen::Vector2d Point2;

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

enum INTERSECTION_STATUS {
    INIT,
    NONE,
    TOUCH_POINT,
    MORE,
    SAME
};

double getLineLength(const Point &a, const Point &b);
void getLineLength(const Point &a, const Point &b, double &out);
unsigned int factorial(const unsigned int &n);
unsigned int combinationWithoutRepetition(const unsigned int &numberOfElements, const unsigned int &numberOfSelectedElements = 2);
bool areNumbersSame(const size_t &i1, const size_t &i2, const size_t &i3, const size_t &i4);
void sphereParametricEquation(const Point &center, const double &radius, const double &s, const double &t, Point &point);
void lineParametricEquation(const Point &p1, const Point &p2, const double &t, Point &point);
void circleParametricEquation(const Point &center, const double &radius, const double &t, const Vector3 &a, const Vector3 &b, Point &point);
bool isTriangle(const double &a, const double &b, const double &c);
bool isPointOnSphereSurface(const Point &point, const Point &center, const double &radius);
void transformPoint(Point &point, const Matrix4 &transform);
void rotateVector(const Vector3 &v, const Vector3 &n, const double &angle, Vector3 &out);
void twoSpheresIntersection(const Point &center1, const double &radius1, const Point &center2, const double &radius2,
                            Point &center, double &radius, Vector3 &v1, Vector3 &v2, INTERSECTION_STATUS &status);
void circleSphereIntersection(const Point &centerCircle, const double &radiusCircle, const Vector3 &v1, const Vector3 &v2,
                              const Point &centerSphere, const double &radiusSphere,
                              std::vector<Point> &points, INTERSECTION_STATUS &status);

#endif //SRC_MATH_H
