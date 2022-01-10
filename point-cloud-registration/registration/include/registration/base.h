#ifndef SRC_BASE_H
#define SRC_BASE_H

#include <registration/math.h>

class Base {
public:
    Base();
    bool setBase(const Point &p1, const Point &p2, const Point &p3, const Point &p4, const double &range = 0);
    [[nodiscard]] const std::vector<Point> &getPoints() const;
    [[nodiscard]] const std::vector<double> &getDescriptors() const;
    const Transform &getFrame() const;

private:
    std::vector<Point> points;
    std::vector<double> descriptors;
    Transform frame;
};

#endif //SRC_BASE_H