#include <localization/matrix.h>

Matrix::Matrix(double leafSize) :
    width(0),
    height(0),
    leafSize(leafSize) {}

void Matrix::fromSet(const Set &set) {
    long index, newSize;
    this->offset = set.getMinBoundingBox();
    this->width = static_cast<long>(ceil(set.getWidth() / this->leafSize));
    this->height = static_cast<long>(ceil(set.getHeight() / this->leafSize));
    newSize = this->width * this->height;
    for (auto &voxel : this->data) { voxel.clear(); }
    if (newSize > this->data.size()) {
        ROS_INFO("Matrix is resizing...");
        this->data.resize(newSize - this->data.size());
    }
    for (auto &point : set.getPointCloud()->points) {
        this->indexFromPoint(point, index);
        this->data[index].push_back(&point);
    }
}

void Matrix::getPoints(const Point &point, double distance, std::vector<Point*> points) {
    long index;
    points.clear();
    this->indexFromPoint(point, index);
    ROS_INFO("point x:%f, y:%f, z:%f", point.x, point.y, point.z);
    for (auto &point2 : this->data[index]) {
        ROS_INFO("found points x:%f, y:%f, z:%f", point2->x, point2->y, point2->z);
    }
    ROS_INFO("index: %ld", index);
}

void Matrix::getMatrixCoordinates(const Point &point, MatrixCoordinate &coordinate) {
    coordinate.x = static_cast<long>(floor((point.x - this->offset.x) / this->leafSize));
    coordinate.y = static_cast<long>(floor((point.y - this->offset.y) / this->leafSize));
}

void Matrix::indexFromPoint(const Point &point, long &index) {
    this->getMatrixCoordinates(point, this->coordinateHelper);
    index = this->coordinateHelper.x + (this->coordinateHelper.y * this->width);
}
