#include <registration/math.h>

double getLineLength(const Point &a, const Point &b) {
    double out;
    getLineLength(a, b, out);
    return out;
}

void getLineLength(const Point &a, const Point &b, double &out) {
    out = sqrt((a - b).dot(a - b));
}

int sumToZero(const int &start) {
    int a = start, sum = 0;
    while (a != 0) {
        sum += a;
        a--;
    }
    return sum;
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
    point = center + (radius * sin(t) * a.normalized() + radius * cos(t) * a.cross(b).cross(a).normalized());
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

void twoSpheresIntersection(const Point &center1, const double &radius1, const Point &center2, const double &radius2,
                            Point &center, double &radius, Vector3 &v1, Vector3 &v2, INTERSECTION_STATUS &status) {
    // TODO check if centers are not equal
    const double centersDistance = getLineLength(center1, center2);
    float t;

    if (radius1 + radius2 - centersDistance == 0) {
        t = static_cast<float>(radius1 / centersDistance);
        lineParametricEquation(center1, center2, t, center);
        status = INTERSECTION_STATUS::TOUCH_POINT;
    } else if (isTriangle(radius1, radius2, centersDistance)) {
        const Vector3 a(center1.x(), center1.y(), center1.z());
        const Vector3 b(center2.x(), center2.y(), center2.z());
        const Vector3 c(center2.x() - center1.x(), center2.y() - center1.y(), center2.z() - center1.z());
        const double alfa = acos((pow(centersDistance, 2) + pow(radius1, 2) - pow(radius2, 2)) / (2 * radius1 * centersDistance));     // Law of cosines

        t = static_cast<float>((radius1 * cos(alfa)) / centersDistance);
        radius = radius1 * sin(alfa);
        v1 = a.cross(b);
        v2 = v1.cross(c);
        lineParametricEquation(center1, center2, t, center);
        status = INTERSECTION_STATUS::MORE;
    } else {
        status = INTERSECTION_STATUS::NONE;
    }
}

void circleSphereIntersection(const Point &centerCircle, const double &radiusCircle, const Vector3 &v1, const Vector3 &v2,
                              const Point &centerSphere, const double &radiusSphere,
                              std::vector<Point> &points, INTERSECTION_STATUS &status) {
    const Vector3 c1(centerCircle.x() - centerSphere.x(),  centerCircle.y() - centerSphere.y(),  centerCircle.z() - centerSphere.z());
    Vector3 n = v1.cross(v2);
    n.normalize();
    double d = n.dot(c1);
    if (fabs(d) > radiusSphere) {
        status = INTERSECTION_STATUS::NONE;
    } else if (fabs(d) == 0) {
        // TODO
        std::wcerr << "Circle Sphere Touch Point !!!!" << std::endl;
        status = INTERSECTION_STATUS::TOUCH_POINT;
    } else if (fabs(d) < radiusSphere) {
        const Point centerCircleSphere(static_cast<float>(centerSphere.x() + (d * n.x())),
                                       static_cast<float>(centerSphere.y() + (d * n.y())),
                                       static_cast<float>(centerSphere.z() + (d * n.z())));
        const double radiusCircleSphere = sqrt(pow(radiusSphere, 2) - pow(d, 2));
        const double centersDistance = getLineLength(centerCircleSphere, centerCircle);
        const double alfa = acos((pow(centersDistance, 2) + pow(radiusCircle, 2) - pow(radiusCircleSphere, 2)) /  (2 * radiusCircle * centersDistance));     // Law of cosines
        Point intersection;
        Vector3 intersectionVector;
        Vector3 c2(centerCircleSphere.x() - centerCircle.x(),
                   centerCircleSphere.y() - centerCircle.y(),
                   centerCircleSphere.z() - centerCircle.z());

        c2.normalize();
        rotateVector(c2, n, alfa, intersectionVector);
        intersectionVector = intersectionVector * radiusCircle;
        intersection = {intersectionVector.x() + centerCircle.x(),
                        intersectionVector.y() + centerCircle.y(),
                        intersectionVector.z() + centerCircle.z()};
        points.push_back(intersection);
        rotateVector(c2, n, -alfa, intersectionVector);
        intersectionVector = intersectionVector * radiusCircle;
        intersection = {intersectionVector.x() + centerCircle.x(),
                        intersectionVector.y() + centerCircle.y(),
                        intersectionVector.z() + centerCircle.z()};
        points.push_back(intersection);
        status = INTERSECTION_STATUS::MORE;
    }
}
