#include <localization/base.h>
#include <ros/ros.h>

Base::Base() = default;

bool Base::setBase(const pcl::PointXYZ &p1, const pcl::PointXYZ &p2,
                   const pcl::PointXYZ &p3, const pcl::PointXYZ &p4, double range) {
    this->descriptors.clear();
    this->points.clear();
    this->descriptors.push_back(getLineLength(p1, p2));
    this->descriptors.push_back(getLineLength(p1, p3));
    this->descriptors.push_back(getLineLength(p1, p4));
    this->descriptors.push_back(getLineLength(p2, p3));
    this->descriptors.push_back(getLineLength(p2, p4));
    this->descriptors.push_back(getLineLength(p3, p4));
    if ((std::accumulate(this->descriptors.begin(), this->descriptors.end(),
                         decltype(this->descriptors)::value_type(0)) / (double)this->descriptors.size()) >= range){
        this->points.push_back(p1);
        this->points.push_back(p2);
        this->points.push_back(p3);
        this->points.push_back(p4);
        return true;
    }
    return false;
}

const std::vector<pcl::PointXYZ> &Base::getPoints() {
    return this->points;
}
