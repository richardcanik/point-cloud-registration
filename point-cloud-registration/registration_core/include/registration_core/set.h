#ifndef SRC_SET_H
#define SRC_SET_H

#include <registration_core/math.h>
#include <cfloat>

class Set {
public:
    Set();
    [[nodiscard]] const double &getWidth() const;
    [[nodiscard]] const double &getHeight() const;
    [[nodiscard]] const double &getDepth() const;
    [[nodiscard]] const Point &getMinBoundingBox() const;
    [[nodiscard]] const Point &getMaxBoundingBox() const;
    [[nodiscard]] const std::vector<Point> &getSet() const;
    void setSet(const std::vector<Point> &inputSet);

private:
    void computeBoundingBox();

    std::vector<Point> set;
    double width;
    double height;
    double depth;
    Point minBoundingBox;
    Point maxBoundingBox;
};

#endif //SRC_SET_H
