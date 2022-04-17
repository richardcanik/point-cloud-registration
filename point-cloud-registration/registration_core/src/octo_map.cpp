#include <registration_core/octo_map.h>

OctoMap::OctoMap(double leafSize) :
    width(0),
    height(0),
    depth(0),
    leafSize(leafSize),
    offset(nullptr) {}

void OctoMap::fromSet(const Set &set) {
    this->width = static_cast<long>(ceil(set.getWidth() / this->leafSize)) + 1;
    this->height = static_cast<long>(ceil(set.getHeight() / this->leafSize)) + 1;
    this->depth = static_cast<long>(ceil(set.getDepth() / this->leafSize)) + 1;
    this->minBoundingBox = set.getMinBoundingBox();
    this->maxBoundingBox = set.getMaxBoundingBox();
    this->offset = &this->minBoundingBox;

    const size_t newSize = this->width * this->height * this->depth;
    size_t index;

    for (auto &voxel : this->voxels)
        voxel.clear();
    if (newSize > this->voxels.size()) {
        std::cout << "Octo map is resizing... " << std::endl;
        this->voxels.resize(newSize - this->voxels.size());  // TODO check if size is in possible range
    }
    for (auto &point : set.getSet()) {
        this->point2VoxelIndex(point, index);
        this->voxels[index].push_back(&point);
    }
    std::cout << "Octo map was filled" << std::endl;
}

void OctoMap::getPoints(const std::vector<Condition*> &conditions, const double &distanceThreshold, std::vector<const Point*> &points) {
    points.clear();
    switch(conditions.size()) {
        case 1:
            this->getPointsFromSphereSurface(conditions, distanceThreshold, points);
            break;
        case 2:
            this->getPointsFrom2SpheresIntersection(conditions, distanceThreshold, points);
            break;
        case 3:
            this->getPointsFrom3SpheresIntersection(conditions, distanceThreshold, points);
            break;
        default:
            std::cerr << "No valid number of conditions" << conditions.size() << std::endl;
    }
}

void OctoMap::getPointsFromSphereSurface(const std::vector<Condition*> &conditions, const double &distanceThreshold, std::vector<const Point*> &points) {
    double radiusLayer, a, b, c;
    int i, j, k;
    Point point;

    // TODO move to math.h + explicit test coverage + test if each point is unique
    for (a = 0; a <= *conditions[0]->descriptor * cos(M_PI_4); a = a + this->leafSize) {
        radiusLayer = sqrt(pow(*conditions[0]->descriptor, 2) - pow(a, 2));
        for (b = 0; b <= radiusLayer * cos(M_PI_4); b = b + this->leafSize) {
            c = sqrt(pow(radiusLayer, 2) - pow(b, 2));
            for (i = -1; i <= 1; i = i + 2)
                for (j = -1; j <= 1; j = j + 2)
                    for (k = -1; k <= 1; k = k + 2) {
                        this->verifyPoint(*conditions[0]->point + Point(a * i, b * j, c * k), conditions, distanceThreshold, points);
                        this->verifyPoint(*conditions[0]->point + Point(b * i, c * j, a * k), conditions, distanceThreshold, points);
                        this->verifyPoint(*conditions[0]->point + Point(c * i, b * j, a * k), conditions, distanceThreshold, points);
                    }
        }
    }
}

void OctoMap::getPointsFrom2SpheresIntersection(const std::vector<Condition*> &conditions, const double &distanceThreshold, std::vector<const Point*> &points) {
    INTERSECTION_STATUS status;
    Point center, point;
    double radius;
    Vector3 v1, v2;
    double s;

    twoSpheresIntersection(*conditions[0]->point, *conditions[0]->descriptor, *conditions[1]->point, *conditions[1]->descriptor, center, radius, v1, v2, status);
    switch(status) {
        case INTERSECTION_STATUS::SAME:
            this->getPointsFromSphereSurface(conditions, distanceThreshold, points);
            break;
        case INTERSECTION_STATUS::TOUCH_POINT:
            this->verifyPoint(center, conditions, distanceThreshold, points);
            break;
        case INTERSECTION_STATUS::MORE:
            // TODO move to math.h + explicit test coverage
            for (s = 0; s < 2 * M_PI; s = s + tan(this->leafSize / radius)) {
                circleParametricEquation(center, radius, s, v1, v2, point);
                this->verifyPoint(point, conditions, distanceThreshold, points);
            }
            break;
        case INTERSECTION_STATUS::NONE:
            break;
        default:
            throw std::invalid_argument("Invalid status");
    }
}

void OctoMap::getPointsFrom3SpheresIntersection(const std::vector<Condition*> &conditions, const double &distanceThreshold, std::vector<const Point*> &points) {
    INTERSECTION_STATUS status1, status2;
    std::vector<Point> intersections;
    Vector3 v1, v2;
    double radius, s;
    Point center, point;

    twoSpheresIntersection(*conditions[1]->point, *conditions[1]->descriptor, *conditions[2]->point, *conditions[2]->descriptor, center, radius, v1, v2, status1);
    switch(status1) {
        case INTERSECTION_STATUS::SAME:
            this->getPointsFrom2SpheresIntersection(conditions, distanceThreshold, points);
            break;
        case INTERSECTION_STATUS::TOUCH_POINT:
            this->verifyPoint(center, conditions, distanceThreshold, points);
        case INTERSECTION_STATUS::MORE:
            circleSphereIntersection(center, radius, v1, v2, *conditions[0]->point, *conditions[0]->descriptor, intersections, status2);
            switch(status2) {
                case INTERSECTION_STATUS::SAME:
                    for (s = 0; s < 2 * M_PI; s = s + tan(this->leafSize / radius)) {
                        circleParametricEquation(center, radius, s, v1, v2, point);
                        this->verifyPoint(point, conditions, distanceThreshold, points);
                    }
                    break;
                case INTERSECTION_STATUS::TOUCH_POINT:
                case INTERSECTION_STATUS::MORE:
                    for (auto &intersection : intersections) {
                        this->verifyPoint(intersection, conditions, distanceThreshold, points);

//                        VoxelCoordinate voxel;
//                        this->point2Coordinate(intersection, voxel);
//                        std::cout << "intersection: " << voxel.x << " " << voxel.y << " " << voxel.z << std::endl;

                    }
                    break;
                case INTERSECTION_STATUS::NONE:
                    break;
                default:
                    throw std::invalid_argument("Invalid status");
            }
            break;
        case INTERSECTION_STATUS::NONE:
            break;
        default:
            throw std::invalid_argument("Invalid status");
    }
//    std::cout << "status1: " << status1 << std::endl;
//    std::cout << "status2: " << status2 << std::endl;
}

void OctoMap::checkVoxelAndPushPoint(const std::vector<Condition*> &conditions, const size_t &index, const double &distanceThreshold, std::vector<const Point*> &points) {
    bool flag;

    if (index >= 0 && index < this->voxels.size()) {
        for (auto &candidate : this->voxels[index]) {
            if (std::find(points.begin(), points.end(), candidate) == points.end()) {
                flag = true;
                for (auto &condition : conditions) {
                    if (abs(getLineLength(*condition->point, *candidate) - *condition->descriptor) > distanceThreshold) {
                        flag = false;
                        break;
                    }
                }
                if (flag)
                    points.push_back(candidate);
            }
        }
    }
}

void OctoMap::point2VoxelIndex(const Point &point, size_t &index) {
    VoxelCoordinate voxel;

    this->point2Coordinate(point, voxel);
    this->coordinate2VoxelIndex(voxel, index);
}

void OctoMap::point2Coordinate(const Point &point, VoxelCoordinate &coordinate) {
    coordinate.x = floor((point.x() - this->offset->x()) / this->leafSize);
    coordinate.y = floor((point.y() - this->offset->y()) / this->leafSize);
    coordinate.z = floor((point.z() - this->offset->z()) / this->leafSize);
}

void OctoMap::coordinate2VoxelIndex(const VoxelCoordinate &coordinate, size_t &index) const {
    if (coordinate.x >= this->width || coordinate.y >= this->height || coordinate.z >= this->depth || coordinate.x < 0 || coordinate.y < 0 || coordinate.z < 0)
        index = -1;
    else
        index = coordinate.x + (coordinate.y * this->width) + (coordinate.z * this->width * this->height);
}

void OctoMap::verifyPoint(const Point &point, const std::vector<Condition*> &conditions, const double &distanceThreshold, std::vector<const Point*> &points) {
    size_t index;
    VoxelCoordinate voxel1, voxel2;
    int x, y, z;

    this->point2Coordinate(point, voxel1);
    for (x = -1; x <=1; x++) {
        for (y = -1; y <=1; y++) {
            for (z = -1; z <=1; z++) {
                voxel2 = {voxel1.x + x, voxel1.y + y, voxel1.z + z};
                this->coordinate2VoxelIndex(voxel2, index);
                this->checkVoxelAndPushPoint(conditions, index, distanceThreshold, points);
            }
        }
    }
}
