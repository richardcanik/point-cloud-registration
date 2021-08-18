#include <localization/octo_map.h>

OctoMap::OctoMap(double leafSize) :
    width(0),
    height(0),
    depth(0),
    leafSize(leafSize) {}

void OctoMap::fromSet(const Set &set) {
    long index, newSize;
    this->offset = set.getMinBoundingBox();
    this->width = static_cast<long>(ceil(set.getWidth() / this->leafSize));
    this->height = static_cast<long>(ceil(set.getHeight() / this->leafSize));
    this->depth = static_cast<long>(ceil(set.getDepth() / this->leafSize));
    newSize = this->width * this->height * this->depth;
    for (auto &voxel : this->data) {
        voxel.clear();
    }
    if (newSize > this->data.size()) {
        ROS_INFO("Octo map is resizing...");
        this->data.resize(newSize - this->data.size());
    }
    ROS_INFO("Octo map size %ld", this->data.size());
    for (auto &point : set.getPointCloud()->points) {
        this->point2VoxelIndex(point, index);
        this->data[index].push_back(&point);
    }
    ROS_INFO("Octo map was created");
}

void OctoMap::getPoints(const std::vector<Condition*> &conditions, const double &distanceThreshold, std::vector<Point*> &points) {
    points.clear();
    if (!conditions.empty()) {
        const double *radius = conditions[0]->descriptor;
        const Point *center = conditions[0]->point;
        double step1 = M_PI_4l / (*radius / this->leafSize), step2, t, s;
        bool pushFlag;
        long index;
        Point p;
        for (t = 0.0; t <= M_PIl; t += step1) {
            step2 = M_PI_4l / ((*radius * sin(t)) / this->leafSize);
            for (s = 0.0; s < 2.0 * M_PIl; s += step2) {
                p.x = center->x + static_cast<float>(*radius * cos(s) * sin(t));
                p.y = center->y + static_cast<float>(*radius * sin(s) * sin(t));
                p.z = center->z + static_cast<float>(*radius * cos(t));
                this->point2VoxelIndex(p, index);
                if (index >= 0) {
                    for (auto &candidate : this->data[index]) {
                        pushFlag = true;
                        for (auto condition : conditions) {
                            if (fabs(getLineLength(*condition->point, *candidate) - *condition->descriptor) > distanceThreshold) {
                                pushFlag = false;
                                break;
                            }
                        }
                        if (pushFlag) {
                            points.push_back(candidate);
                        }
                    }
                }
            }
        }
    } else {
        ROS_ERROR("No condition is defined");
    }
}

void OctoMap::point2VoxelIndex(const Point &point, long &index) {
    this->point2Coordinate(point, this->coordinateHelper);
    this->coordinate2VoxelIndex(this->coordinateHelper, index);
}

void OctoMap::point2Coordinate(const Point &point, VoxelCoordinate &coordinate) {
    coordinate.x = static_cast<long>(floor((point.x - this->offset.x) / this->leafSize));
    coordinate.y = static_cast<long>(floor((point.y - this->offset.y) / this->leafSize));
    coordinate.z = static_cast<long>(floor((point.z - this->offset.z) / this->leafSize));
}

void OctoMap::coordinate2VoxelIndex(const VoxelCoordinate &coordinate, long &index) const {
    if (coordinate.x >= this->width || coordinate.y >= this->height || coordinate.z >= this->depth ||
        coordinate.x < 0 || coordinate.y < 0 || coordinate.z < 0) {
        index = -1;
    } else {
        index = coordinate.x + (coordinate.y * this->width) + (coordinate.z * this->width * this->height);
    }
}
