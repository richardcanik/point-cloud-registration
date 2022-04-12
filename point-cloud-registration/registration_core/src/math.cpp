#include <registration_core/math.h>

double getLineLength(const Point &a, const Point &b) {
    double out;
    getLineLength(a, b, out);
    return out;
}

void getLineLength(const Point &a, const Point &b, double &out) {
    out = sqrt((a - b).squaredNorm());
}

unsigned int factorial(const unsigned int &n) {
    if (n == 0) return 1;
    return n * factorial(n - 1);
}

unsigned int combinationWithoutRepetition(const unsigned int &numberOfElements, const unsigned int &numberOfSelectedElements) {
    if (numberOfElements < numberOfSelectedElements) throw std::invalid_argument("The Number of elements is lower than the number of selected elements");
    return factorial(numberOfElements) / (factorial(numberOfSelectedElements) * factorial(numberOfElements - numberOfSelectedElements));
}

bool areNumbersSame(const size_t &i1, const size_t &i2, const size_t &i3, const size_t &i4) {
    return !(i1 == i2 || i1 == i3 || i1 == i4 || i2 == i3 || i2 == i4 || i3 == i4);
}

void sphereParametricEquation(const Point &center, const double &radius, const double &s, const double &t, Point &point) {
    point = center + Point(static_cast<double>(radius * cos(s) * sin(t)), static_cast<double>(radius * sin(s) * sin(t)), static_cast<double>(radius * cos(t)));
}

void lineParametricEquation(const Point &p1, const Point &p2, const double &t, Point &point) {
    point = p1 + (t * (p2 - p1));
}

void circleParametricEquation(const Point &center, const double &radius, const double &t, const Vector3 &a, const Vector3 &b, Point &point) {
    point = center + (radius * sin(t) * a.normalized() + radius * cos(t) * a.cross(b).cross(a).normalized());
}

bool isTriangle(const double &a, const double &b, const double &c) {
    return (a + b > c && a + c > b && b + c > a);
}

void transformPoint(Point &point, const Matrix4 &transform) {
    Vector4 position;
    position = point.head(4);
    position(3) = 1;
    point = (transform * position).head(3);
}

void rotateVector(const Vector3 &v, const Vector3 &n, const double &angle, Vector3 &out) {
    out = (v * cos(angle)) + (n.cross(v) * sin(angle)) + (n * n.dot(v)) * (1 - cos(angle));
}

void twoSpheresIntersection(const Point &center1, const double &radius1, const Point &center2, const double &radius2,
                            Point &center, double &radius, Vector3 &v1, Vector3 &v2, INTERSECTION_STATUS &status) {
    status = INTERSECTION_STATUS::INIT;
    center = v1 = v2 = {0, 0, 0};
    radius = -1;
    const auto centersDistance = getLineLength(center1, center2);
    double t;

    if (centersDistance == 0 && radius1 == radius2) {
        status = INTERSECTION_STATUS::SAME;
    } else if (radius1 + radius2 - centersDistance == 0) {
        t = static_cast<double>(radius1 / centersDistance);
        lineParametricEquation(center1, center2, t, center);
        status = INTERSECTION_STATUS::TOUCH_POINT;
    } else if (isTriangle(radius1, radius2, centersDistance)) {
        const auto alfa = acos((pow(centersDistance, 2) + pow(radius1, 2) - pow(radius2, 2)) / (2 * radius1 * centersDistance));     // Law of cosines
        t = static_cast<double>((radius1 * cos(alfa)) / centersDistance);
        radius = radius1 * sin(alfa);
        v1 = center1.cross(center2);
        v2 = v1.cross(center2 - center1);
        lineParametricEquation(center1, center2, t, center);
        status = INTERSECTION_STATUS::MORE;
    } else {
        status = INTERSECTION_STATUS::NONE;
    }
}

void circleSphereIntersection(const Point &centerCircle, const double &radiusCircle, const Vector3 &v1, const Vector3 &v2,
                              const Point &centerSphere, const double &radiusSphere,
                              std::vector<Point> &points, INTERSECTION_STATUS &status) {
    status = INTERSECTION_STATUS::INIT;
    points.clear();
    Vector3 n = v1.cross(v2);
    const double planeSphereDistance = abs((centerSphere - centerCircle).dot(n) / sqrt(n.squaredNorm()));

    if (abs(planeSphereDistance) <= radiusSphere) {
        const double r = sqrt(pow(radiusSphere, 2) - pow(planeSphereDistance, 2));
        const Point c = centerSphere + planeSphereDistance * (n / sqrt(n.squaredNorm()));
        const double centerDistance = getLineLength(c, centerCircle);
        if (centerDistance == 0 && radiusCircle == r) {
            status = INTERSECTION_STATUS::SAME;
        } else if (radiusCircle + r - centerDistance == 0 ) {
            const auto t = static_cast<double>(radiusCircle / centerDistance);
            Point p;
            lineParametricEquation(centerCircle, c, t, p);
            points.push_back(p);
            status = INTERSECTION_STATUS::TOUCH_POINT;
        } else if (isTriangle(centerDistance, radiusCircle, r)) {
            const double alfa = acos((pow(centerDistance, 2) + pow(radiusCircle, 2) - pow(r, 2)) /  (2 * radiusCircle * centerDistance));     // Law of cosines
            const Vector3 v = (c - centerCircle).normalized();
            Vector3 vc1, vc2;
            rotateVector(v, n, alfa, vc1);
            rotateVector(v, n, -alfa, vc2);
            vc1 = centerCircle + vc1 * radiusCircle;
            vc2 = centerCircle + vc2 * radiusCircle;
            points.push_back(vc1);
            points.push_back(vc2);
            status = INTERSECTION_STATUS::MORE;
        } else {
            status = INTERSECTION_STATUS::NONE;
        }
    } else {
        status = INTERSECTION_STATUS::NONE;
    }
}
