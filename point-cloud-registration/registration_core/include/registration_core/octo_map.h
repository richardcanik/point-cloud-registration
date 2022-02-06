#ifndef SRC_OCTO_MAP_H
#define SRC_OCTO_MAP_H

#include <registration_core/set.h>
#include <iostream>

class OctoMap {
public:
    explicit OctoMap(double leafSize);
    void fromSet(const Set &set);
    void getPoints(const std::vector<Condition*> &conditions, const double &distanceThreshold, std::vector<const Point*> &points);

private:
    void getPointsFromSphereSurface(const Point &center, const double &radius, const double &distanceThreshold, std::vector<const Point*> &points);
    void getPointsFrom2SpheresIntersection(const Point &center1, const double &radius1,
                                           const Point &center2, const double &radius2,
                                           const double &distanceThreshold, std::vector<const Point*> &points);
    void getPointsFrom3SpheresIntersection(const Point &center1, const double &radius1,
                                           const Point &center2, const double &radius2,
                                           const Point &center3, const double &radius3,
                                           const double &distanceThreshold, std::vector<const Point*> &points);
    void checkVoxel(const Point &point, const double &length, const long &index, const double &distanceThreshold, std::vector<const Point*> &points);
    void point2VoxelIndex(const Point &point, long &index);
    void point2Coordinate(const Point &point, VoxelCoordinate &coordinate);
    void coordinate2VoxelIndex(const VoxelCoordinate &coordinate, long &index) const;

    long width;
    long height;
    long depth;
    double leafSize;
    Point *offset;
    Point minBoundingBox;
    Point maxBoundingBox;
    std::vector<std::vector<const Point*>> voxels;
    VoxelCoordinate coordinateHelper;
};

#endif //SRC_OCTO_MAP_H