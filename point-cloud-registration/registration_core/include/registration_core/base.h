#ifndef SRC_BASE_H
#define SRC_BASE_H

#include <registration_core/math.h>
#include <numeric>

class Base {
public:
    Base();
    bool setBase(const Point &p1, const Point &p2, const Point &p3, const Point &p4, const double &range = 0);
    [[nodiscard]] const std::vector<Point> &getPoints() const;
    [[nodiscard]] const std::vector<double> &getDescriptors() const;
    [[nodiscard]] const Matrix4 &getFrame() const;

private:
    std::vector<Point> points;
    std::vector<double> descriptors;
    Matrix4 frame;
    Vector3 v, i, j, k;
};

#endif //SRC_BASE_H
