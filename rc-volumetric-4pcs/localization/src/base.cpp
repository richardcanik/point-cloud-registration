#include <localization/base.h>
#include <ros/ros.h>

Base::Base() = default;

bool Base::setBase(const Point &p1, const Point &p2, const Point &p3, const Point &p4, double range) {
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

        Eigen::Vector3f v, i, j, k;

        // Set Vectors
        i(0) = p2.x - p1.x;
        i(1) = p2.y - p1.y;
        i(2) = p2.z - p1.z;
        v(0) = p3.x - p1.x;
        v(1) = p3.y - p1.y;
        v(2) = p3.z - p1.z;

        // Get normal vector
        k(0) = i.y() * v.z() - i.z() * v.y();
        k(1) = i.z() * v.x() - i.x() * v.z();
        k(2) = i.x() * v.y() - i.y() * v.x();

        j(0) = k.y() * i.z() - k.z() * i.y();
        j(1) = k.z() * i.x() - k.x() * i.z();
        j(2) = k.x() * i.y() - k.y() * i.x();

        i.normalize();
        j.normalize();
        k.normalize();

        // Rotation
        this->frame(0, 0) = i.x();
        this->frame(1, 0) = i.y();
        this->frame(2, 0) = i.z();
        this->frame(0, 1) = j.x();
        this->frame(1, 1) = j.y();
        this->frame(2, 1) = j.z();
        this->frame(0, 2) = k.x();
        this->frame(1, 2) = k.y();
        this->frame(2, 2) = k.z();
        // Translation
        this->frame(0, 3) = p1.x;
        this->frame(1, 3) = p1.y;
        this->frame(2, 3) = p1.z;
        this->frame(3, 3) = 1;
        return true;
    }
    return false;
}

const std::vector<Point> &Base::getPoints() {
    return this->points;
}

const std::vector<double> &Base::getDescriptors() {
    return this->descriptors;
}

const Transform &Base::getFrame() {
    return this->frame;
}
