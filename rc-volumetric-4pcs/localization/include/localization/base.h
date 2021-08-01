#ifndef SRC_BASE_H
#define SRC_BASE_H

#include <pcl/io/ply_io.h>
#include <localization/helper.h>

class Base {
public:
    Base();
    bool setBase(const Point &p1, const Point &p2, const Point &p3, const Point &p4, double range = 0);
    const std::vector<Point> &getPoints();
    const std::vector<double> &getDescriptors();

private:
    std::vector<Point> points;
    std::vector<double> descriptors;
};

#endif //SRC_BASE_H
