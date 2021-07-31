#ifndef SRC_BASE_H
#define SRC_BASE_H

#include <pcl/io/ply_io.h>
#include <localization/helper.h>

class Base {
public:
    Base();
    bool setBase(const pcl::PointXYZ &p1, const pcl::PointXYZ &p2,
                 const pcl::PointXYZ &p3, const pcl::PointXYZ &p4, double range = 0);
    const std::vector<pcl::PointXYZ> &getPoints();

private:
    std::vector<pcl::PointXYZ> points;
    std::vector<double> descriptors;
};

#endif //SRC_BASE_H
