#include <localization/octo_map.h>

OctoMap::OctoMap(ros::NodeHandle &nodeHandle, double leafSize) :
    width(0),
    height(0),
    depth(0),
    leafSize(leafSize),
    offset(nullptr),
    publisherPoints1(nodeHandle.advertise<visualization_msgs::Marker>("/octo_map/points1", 1)),
    publisherPoints2(nodeHandle.advertise<visualization_msgs::Marker>("/octo_map/points2", 1)),
    publisherPoints3(nodeHandle.advertise<visualization_msgs::Marker>("/octo_map/points3", 1)),
    publisherPoints4(nodeHandle.advertise<visualization_msgs::Marker>("/octo_map/points4", 1)) {}

void OctoMap::fromSet(const Set &set) {
    this->width = static_cast<long>(ceil(set.getWidth() / this->leafSize));
    this->height = static_cast<long>(ceil(set.getHeight() / this->leafSize));
    this->depth = static_cast<long>(ceil(set.getDepth() / this->leafSize));
    this->minBoundingBox = set.getMinBoundingBox();
    this->maxBoundingBox = set.getMaxBoundingBox();
    this->offset = &this->minBoundingBox;

    long newSize = this->width * this->height * this->depth;
    long index;

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

void OctoMap::getPoints(const std::vector<Condition*> &conditions, const double &distanceThreshold,
                        std::vector<Point*> &points) {
    points.clear();
    switch(conditions.size()) {
        case 1:
            ROS_INFO("Number of condition 1");
            this->getPointsFromSphereSurface(*conditions[0]->point, *conditions[0]->descriptor, distanceThreshold,
                                             points);
            break;
        case 2:
            ROS_INFO("Number of condition 2");
            this->getPointsFrom2SpheresIntersection(conditions, distanceThreshold, points);
            break;
        default:
            ROS_ERROR("No valid number of conditions. %ld", conditions.size());
    }
}

void OctoMap::getPointsFromSphereSurface(const Point &center, const double &radius, const double &distanceThreshold,
                                         std::vector<Point*> &points) {
    // TODO check if the center of a sphere is in the bounding-box
    const double dxMax = fabs(this->maxBoundingBox.x - center.x);
    const double dxMin = fabs(this->minBoundingBox.x - center.x);
    const double dyMax = fabs(this->maxBoundingBox.y - center.y);
    const double dyMin = fabs(this->minBoundingBox.y - center.y);
    const double dzMax = fabs(this->maxBoundingBox.z - center.z);
    const double dzMin = fabs(this->minBoundingBox.z - center.z);
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
                if (index >= 0) {
                    for (auto &candidate : this->data[index]) {
                        if (fabs(getLineLength(center, *candidate) - radius) > distanceThreshold) {
                            points.push_back(candidate);
                        }
                    }
                }
            }
        }
    }
}

void OctoMap::getPointsFrom2SpheresIntersection(const std::vector<Condition*> &conditions,
                                                const double &distanceThreshold, std::vector<Point*> &points) {
    if (conditions.size() == 2) {
        // TODO check if centers are not equal
        const Point *center1 = conditions[0]->point;
        const Point *center2 = conditions[1]->point;
        const double *radius1 = conditions[0]->descriptor;
        const double *radius2 = conditions[1]->descriptor;
        const double centersDistance = getLineLength(*center1, *center2);
        const double k = (*radius1 + *radius2) - centersDistance;
        if (k == 0) {
            ROS_WARN("It is really equal, It is funny :D");
        } else if (k > 0) {
            const double alfa = acos((pow(centersDistance, 2) + pow(*radius1, 2) - pow(*radius2, 2)) /
                                     (2 * *radius1 * centersDistance));     // Law of cosines
            const auto t = static_cast<float>((*radius1 * cos(alfa)) / centersDistance);
            const double radiusCircle = *radius1 * sin(alfa);
            const Vector a(center1->x, center1->y, center1->z);
            const Vector b(center2->x, center2->y, center2->z);
            const Vector c(center2->x - center1->x, center2->y - center1->y, center2->z - center1->z);
            const Vector v1 = a.cross(b);
            const Vector v2 = v1.cross(c);
            Point centerCircle, point;
            long index;
            std::vector<Point> points1, points2, points3, points4;

            lineParametricEquation(*center1, *center2, t, centerCircle);
            for (double s = 0; s < 2*M_PI; s = s + 0.05) {
                circleParametricEquation(centerCircle, radiusCircle, s, v1, v2, point);
                this->point2VoxelIndex(point, index);
                if (index >= 0) {
                    for (auto &candidate : this->data[index]) {
                        if (fabs(getLineLength(centerCircle, *candidate) - radiusCircle) > distanceThreshold) {
                            points.push_back(candidate);
                        }
                    }
                }
                points4.push_back(point);
            }

            // Publish spheres
            for (double e = 0; e < M_PI; e = e + 0.05) {
                for (double s = 0; s < 2*M_PI; s = s + 0.05) {
                    sphereParametricEquation(*center1, *radius1, s, e, point);
                    points1.push_back(point);
                    sphereParametricEquation(*center2, *radius2, s, e, point);
                    points2.push_back(point);
                }
            }
            points3.push_back(*center1);
            points3.push_back(*center2);
            points3.push_back(centerCircle);
            publishPoints(points1, this->publisherPoints1);
            publishPoints(points2, this->publisherPoints2);
            publishPoints(points3, this->publisherPoints3, 0, 1, 0, 3);
            publishPoints(points4, this->publisherPoints4, 0, 0, 1);
        } else {
            ROS_WARN("There is no intersection");
        }
    } else {
        ROS_ERROR("Wrong number of conditions (%ld) from getPointsFrom2SpheresIntersection.", conditions.size());
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
