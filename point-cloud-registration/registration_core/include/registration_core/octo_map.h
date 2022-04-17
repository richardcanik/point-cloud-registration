#ifndef SRC_OCTO_MAP_H
#define SRC_OCTO_MAP_H

#include <registration_core/set.h>
#include <iostream>

class OctoMap {
public:
    explicit OctoMap(double leafSize);
    void fromSet(const Set &set);
    void getPoints(const std::vector<Condition*> &conditions, const double &distanceThreshold, std::vector<const Point*> &points);
    void point2Coordinate(const Point &point, VoxelCoordinate &coordinate);

private:
    void getPointsFromSphereSurface(const std::vector<Condition*> &conditions, const double &distanceThreshold, std::vector<const Point*> &points);
    void getPointsFrom2SpheresIntersection(const std::vector<Condition*> &conditions, const double &distanceThreshold, std::vector<const Point*> &points);
    void getPointsFrom3SpheresIntersection(const std::vector<Condition*> &conditions, const double &distanceThreshold, std::vector<const Point*> &points);
    void checkVoxelAndPushPoint(const std::vector<Condition*> &conditions, const size_t &index, const double &distanceThreshold, std::vector<const Point*> &points);
    void point2VoxelIndex(const Point &point, size_t &index);
    void coordinate2VoxelIndex(const VoxelCoordinate &coordinate, size_t &index) const;
    void verifyPoint(const Point &point, const std::vector<Condition*> &conditions, const double &distanceThreshold, std::vector<const Point*> &points);

    size_t width;
    size_t height;
    size_t depth;
    double leafSize;
    Point *offset;
    Point minBoundingBox;
    Point maxBoundingBox;
    std::vector<std::vector<const Point*>> voxels;
};

#endif //SRC_OCTO_MAP_H
