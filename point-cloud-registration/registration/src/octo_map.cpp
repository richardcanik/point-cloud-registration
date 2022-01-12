#include <registration/octo_map.h>

OctoMap::OctoMap(double leafSize) :
    width(0),
    height(0),
    depth(0),
    leafSize(leafSize),
    offset(nullptr) {}

void OctoMap::fromSet(const Set &set) {
    this->width = static_cast<long>(ceil(set.getWidth() / this->leafSize));
    this->height = static_cast<long>(ceil(set.getHeight() / this->leafSize));
    this->depth = static_cast<long>(ceil(set.getDepth() / this->leafSize));
    this->minBoundingBox = set.getMinBoundingBox();
    this->maxBoundingBox = set.getMaxBoundingBox();
    this->offset = &this->minBoundingBox;

    long newSize = this->width * this->height * this->depth;
    long index;

    for (auto &voxel : this->voxels) {
        voxel.clear();
    }
    if (newSize > this->voxels.size()) {
        std::cout << "Octo map is resizing..." << std::endl;
        this->voxels.resize(newSize - this->voxels.size());
    }
    std::cout << "Octo map size is " << this->voxels.size() << std::endl;
    for (const auto & point : set.getSet()) {
        this->point2VoxelIndex(point, index);
        this->voxels[index].push_back(&point);
    }
    std::cout << "Octo map was created" << std::endl;
}

void OctoMap::getPoints(const std::vector<Condition*> &conditions, const double &distanceThreshold, std::vector<const Point*> &points) {
    points.clear();
    switch(conditions.size()) {
        case 1:
            this->getPointsFromSphereSurface(*conditions[0]->point, *conditions[0]->descriptor,
                                             distanceThreshold,points);
            break;
        case 2:
            this->getPointsFrom2SpheresIntersection(*conditions[0]->point, *conditions[0]->descriptor,
                                                    *conditions[1]->point, *conditions[1]->descriptor,
                                                    distanceThreshold, points);
            break;
        case 3:
            this->getPointsFrom3SpheresIntersection(*conditions[0]->point, *conditions[0]->descriptor,
                                                    *conditions[1]->point, *conditions[1]->descriptor,
                                                    *conditions[2]->point, *conditions[2]->descriptor,
                                                    distanceThreshold, points);
            break;
        default:
            std::cerr << "No valid number of conditions" << conditions.size() << std::endl;
    }
}

void OctoMap::getPointsFromSphereSurface(const Point &center, const double &radius, const double &distanceThreshold, std::vector<const Point*> &points) {
    // TODO check if the center of a sphere is inside the bounding-box
    const double dxMax = fabs(this->maxBoundingBox.x() - center.x());
    const double dxMin = fabs(this->minBoundingBox.x() - center.x());
    const double dyMax = fabs(this->maxBoundingBox.y() - center.y());
    const double dyMin = fabs(this->minBoundingBox.y() - center.y());
    const double dzMax = fabs(this->maxBoundingBox.z() - center.z());
    const double dzMin = fabs(this->minBoundingBox.z() - center.z());
    double step1 = M_PI_4 / (radius / this->leafSize), step2;
    double radiusLayer, angle, t, s;
    double tStart = 0, tEnd = M_PI;
    std::vector<double> sStart;
    std::vector<double> sEnd;
    size_t quadrant;
    long index;
    Point p;

    if (dzMax < radius) {
        tStart = acos(dzMax / radius);
    }
    if (dzMin < radius) {
        tEnd = M_PI - acos(dzMin / radius);
    }
    for (t = tStart; t <= tEnd; t += step1) {
        radiusLayer = (radius * sin(t));
        step2 = M_PI_4 / ( radiusLayer / this->leafSize);
        sEnd = {M_PI_2, M_PI, M_PI + M_PI_2, 2 * M_PI};
        sStart = {0, M_PI_2, M_PI, M_PI + M_PI_2};
        if (dxMax < radiusLayer) {
            sStart[QUADRANT::ONE] = acos(dxMax / radiusLayer);
            sEnd[QUADRANT::FOUR] = 2.0 * M_PI - sStart[QUADRANT::ONE];
        }
        if (dxMin < radiusLayer) {
            angle = acos(dxMin / radiusLayer);
            sStart[QUADRANT::THREE] = M_PI + angle;
            sEnd[QUADRANT::TWO] = M_PI - angle;
        }
        if (dyMax < radiusLayer) {
            angle = acos(dyMax / radiusLayer);
            sStart[QUADRANT::TWO] = M_PI_2 + angle;
            sEnd[QUADRANT::ONE] = M_PI_2 - angle;
        }
        if (dyMin < radiusLayer) {
            angle = acos(dyMin / radiusLayer);
            sStart[QUADRANT::FOUR] = (M_PI + M_PI_2) + angle;
            sEnd[QUADRANT::THREE] = (M_PI + M_PI_2) - angle;
        }
        for (quadrant = 0; quadrant < 4; quadrant++) {
            for (s = sStart[quadrant]; s < sEnd[quadrant]; s += step2) {
                sphereParametricEquation(center, radius, s, t, p);
                this->point2VoxelIndex(p, index);
                this->checkVoxel(center, radius, index, distanceThreshold, points);
            }
        }
    }
}

void OctoMap::getPointsFrom2SpheresIntersection(const Point &center1, const double &radius1,
                                                const Point &center2, const double &radius2,
                                                const double &distanceThreshold, std::vector<const Point*> &points) {
    Point center, point;
    double radius;
    Vector3 v1, v2;
    int status;
    long index;
    double s;

    twoSpheresIntersection(center1, radius1, center2, radius2, center, radius, v1, v2, status);
    switch(status) {
        case INTERSECTION_STATUS::TOUCH_POINT:
            this->point2VoxelIndex(center, index);
            this->checkVoxel(center1, radius1, index, distanceThreshold, points);
            break;
        case INTERSECTION_STATUS::MORE:
            for (s = 0; s < 2 * M_PI; s += M_PI_4 / (radius / this->leafSize)) {
                circleParametricEquation(center, radius, s, v1, v2, point);
                this->point2VoxelIndex(point, index);
                this->checkVoxel(center, radius, index, distanceThreshold, points);
            }
            break;
        default:
            break;
    }
}

void OctoMap::getPointsFrom3SpheresIntersection(const Point &center1, const double &radius1,
                                                const Point &center2, const double &radius2,
                                                const Point &center3, const double &radius3,
                                                const double &distanceThreshold, std::vector<const Point*> &points) {
    Point center;
    double radius;
    Vector3 v1, v2;
    long index;
    int status1, status2;
    std::vector<Point> intersections;

    twoSpheresIntersection(center1, radius1, center2, radius2, center, radius, v1, v2, status1);
    switch(status1) {
        case INTERSECTION_STATUS::TOUCH_POINT:
            this->point2VoxelIndex(center, index);
            this->checkVoxel(center3, radius3, index, distanceThreshold, points);
            break;
        case INTERSECTION_STATUS::MORE:
            circleSphereIntersection(center, radius, v1, v2, center3, radius3, intersections, status2);
            switch(status2) {
                case INTERSECTION_STATUS::TOUCH_POINT:
                case INTERSECTION_STATUS::MORE:
                    for (auto &intersection : intersections) {
                        this->point2VoxelIndex(intersection, index);
                        this->checkVoxel(center3, radius3, index, distanceThreshold, points);
                    }
                    break;
                default:
                    break;
            }
            break;
        default:
            break;
    }
}

void OctoMap::checkVoxel(const Point &point, const double &length, const long &index, const double &distanceThreshold,
                         std::vector<const Point*> &points) {
    if (index >= 0) {
        for (auto &candidate : this->voxels[index]) {
            if (fabs(getLineLength(point, *candidate) - length) <= distanceThreshold) {
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
    coordinate.x = static_cast<long>(floor((point.x() - this->offset->x()) / this->leafSize));
    coordinate.y = static_cast<long>(floor((point.y() - this->offset->y()) / this->leafSize));
    coordinate.z = static_cast<long>(floor((point.z() - this->offset->z()) / this->leafSize));
}

void OctoMap::coordinate2VoxelIndex(const VoxelCoordinate &coordinate, long &index) const {
    if (coordinate.x >= this->width || coordinate.y >= this->height || coordinate.z >= this->depth ||
        coordinate.x < 0 || coordinate.y < 0 || coordinate.z < 0) {
        index = -1;
    } else {
        index = coordinate.x + (coordinate.y * this->width) + (coordinate.z * this->width * this->height);
    }
}
