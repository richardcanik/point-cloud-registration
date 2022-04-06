#include <registration_core/set.h>

Set::Set() :
    width(0),
    height(0),
    depth(0) {}

const double &Set::getWidth() const {
    return this->width;
}

const double &Set::getHeight() const {
    return this->height;
}

const double &Set::getDepth() const {
    return this->depth;
}

const Point &Set::getMinBoundingBox() const {
    return this->minBoundingBox;
}

const Point &Set::getMaxBoundingBox() const {
    return this->maxBoundingBox;
}

const std::vector<Point> &Set::getSet() const {
    return this->set;
}

void Set::setSet(const std::vector<Point> &inputSet) {
    this->set = inputSet;
    computeBoundingBox();
}

void Set::computeBoundingBox() {
    this->minBoundingBox = {DBL_MAX, DBL_MAX, DBL_MAX};
    this->maxBoundingBox = {-DBL_MAX, -DBL_MAX, -DBL_MAX};
    for (auto &point : this->set) {
        if (point.x() < this->minBoundingBox.x()) {
            this->minBoundingBox.x() = point.x();
        }
        if (point.x() > this->maxBoundingBox.x()) {
            this->maxBoundingBox.x() = point.x();
        }
        if (point.y() < this->minBoundingBox.y()) {
            this->minBoundingBox.y() = point.y();
        }
        if (point.y() > this->maxBoundingBox.y()) {
            this->maxBoundingBox.y() = point.y();
        }
        if (point.z() < this->minBoundingBox.z()) {
            this->minBoundingBox.z() = point.z();
        }
        if (point.z() > this->maxBoundingBox.z()) {
            this->maxBoundingBox.z() = point.z();
        }
    }
    this->width = fabsf(this->maxBoundingBox.x() - this->minBoundingBox.x());
    this->height = fabsf(this->maxBoundingBox.y() - this->minBoundingBox.y());
    this->depth = fabsf(this->maxBoundingBox.z() - this->minBoundingBox.z());
}
