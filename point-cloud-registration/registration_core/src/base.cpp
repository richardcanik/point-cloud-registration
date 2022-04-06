#include <registration_core/base.h>

Base::Base():
    descriptors(combinationWithoutRepetition(4)),
    points(4) {};

bool Base::setBase(const Point &p1, const Point &p2, const Point &p3, const Point &p4, const double &range) {
    // TODO try to use descriptor d^2
    getLineLength(p1, p2, this->descriptors[0]);
    getLineLength(p1, p3, this->descriptors[1]);
    getLineLength(p1, p4, this->descriptors[2]);
    getLineLength(p2, p3, this->descriptors[3]);
    getLineLength(p2, p4, this->descriptors[4]);
    getLineLength(p3, p4, this->descriptors[5]);
    if ((std::accumulate(this->descriptors.begin(), this->descriptors.end(),
                         decltype(this->descriptors)::value_type(0)) / (double)this->descriptors.size()) >= range){
        this->points[0] = p1;
        this->points[1] = p2;
        this->points[2] = p3;
        this->points[3] = p4;
        i = p2 - p1;
        v = p3 - p1;
        k = i.cross(v);
        j = i.cross(k);
        this->frame.setIdentity();
        this->frame.block<3,1>(0,0) = i.normalized();
        this->frame.block<3,1>(0,1) = j.normalized();
        this->frame.block<3,1>(0,2) = k.normalized();
        this->frame.block<3,1>(0,3) = p1;
        return true;
    }
    return false;
}

const std::vector<Point> &Base::getPoints() const {
    return this->points;
}

const std::vector<double> &Base::getDescriptors() const {
    return this->descriptors;
}

const Matrix4 &Base::getFrame() const {
    return this->frame;
}
