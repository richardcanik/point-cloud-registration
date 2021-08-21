#include <localization/octo_map.h>

OctoMap::OctoMap(double leafSize) :
    width(0),
    height(0),
    depth(0),
    leafSize(leafSize) {}

void OctoMap::fromSet(const Set &set) {
    long index, newSize;
    this->minBoundingBox = set.getMinBoundingBox();
    this->maxBoundingBox = set.getMaxBoundingBox();
    this->offset = &this->minBoundingBox;
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
        const Point *center = conditions[0]->point;       // TODO check if the center of a sphere is in the bounding-box
        double step1 = M_PI_4 / (*radius / this->leafSize), step2, t, s;
        double dxMax = fabs(this->maxBoundingBox.x - center->x);
        double dxMin = fabs(this->minBoundingBox.x - center->x);
        double dyMax = fabs(this->maxBoundingBox.y - center->y);
        double dyMin = fabs(this->minBoundingBox.y - center->y);
        double dzMax = fabs(this->maxBoundingBox.z - center->z);
        double dzMin = fabs(this->minBoundingBox.z - center->z);
        double tStart = 0, tEnd = M_PI;
        std::vector<double> sStart;
        std::vector<double> sEnd;
        size_t quadrant;
        if (dzMax < *radius) {
            tStart = acos(dzMax / *radius);
        }
        if (dzMin < *radius) {
            tEnd = M_PI - acos(dzMin / *radius);
        }
        double radiusLayer;
        double helperAngle;
        for (t = tStart; t <= tEnd; t += step1) {
            radiusLayer = (*radius * sin(t));
            step2 = M_PI_4 / ( radiusLayer / this->leafSize);
            sEnd = {M_PI_2, M_PI, M_PI + M_PI_2, 2 * M_PI};
            sStart = {0, M_PI_2, M_PI, M_PI + M_PI_2};
            if (dxMax < radiusLayer) {
                sStart[QUADRANT::ONE] = acos(dxMax / radiusLayer);
                sEnd[QUADRANT::FOUR] = 2.0 * M_PI - sStart[QUADRANT::ONE];
            }
            if (dxMin < radiusLayer) {
                helperAngle = acos(dxMin / radiusLayer);
                sStart[QUADRANT::THREE] = M_PI + helperAngle;
                sEnd[QUADRANT::TWO] = M_PI - helperAngle;
            }
            if (dyMax < radiusLayer) {
                helperAngle = acos(dyMax / radiusLayer);
                sStart[QUADRANT::TWO] = M_PI_2 + helperAngle;
                sEnd[QUADRANT::ONE] = M_PI_2 - helperAngle;
            }
            if (dyMin < radiusLayer) {
                helperAngle = acos(dyMin / radiusLayer);
                sStart[QUADRANT::FOUR] = (M_PI + M_PI_2) + helperAngle;
                sEnd[QUADRANT::THREE] = (M_PI + M_PI_2) - helperAngle;
            }
            for (quadrant = 0; quadrant < 4; quadrant++) {
                for (s = sStart[quadrant]; s < sEnd[quadrant]; s += step2) {
                    this->pointFromSphere(conditions, distanceThreshold, points, t, s, *center, *radius);
                }
            }
        }
    } else {
        ROS_ERROR("No condition is defined");
    }
}


void OctoMap::pointFromSphere(const std::vector<Condition*> &conditions, const double &distanceThreshold,
                               std::vector<Point*> &points, const double &t, const double &s,
                               const Point &center, const double &radius) {
    Point p;
    bool pushFlag;
    long index;
    p.x = center.x + static_cast<float>(radius * cos(s) * sin(t));
    p.y = center.y + static_cast<float>(radius * sin(s) * sin(t));
    p.z = center.z + static_cast<float>(radius * cos(t));
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

void OctoMap::point2VoxelIndex(const Point &point, long &index) {
    this->point2Coordinate(point, this->coordinateHelper);
    this->coordinate2VoxelIndex(this->coordinateHelper, index);
}

void OctoMap::point2Coordinate(const Point &point, VoxelCoordinate &coordinate) {
    coordinate.x = static_cast<long>(floor((point.x - this->offset->x) / this->leafSize));
    coordinate.y = static_cast<long>(floor((point.y - this->offset->y) / this->leafSize));
    coordinate.z = static_cast<long>(floor((point.z - this->offset->z) / this->leafSize));
}

void OctoMap::coordinate2VoxelIndex(const VoxelCoordinate &coordinate, long &index) const {
    if (coordinate.x >= this->width || coordinate.y >= this->height || coordinate.z >= this->depth ||
        coordinate.x < 0 || coordinate.y < 0 || coordinate.z < 0) {
        index = -1;
    } else {
        index = coordinate.x + (coordinate.y * this->width) + (coordinate.z * this->width * this->height);
    }
}
