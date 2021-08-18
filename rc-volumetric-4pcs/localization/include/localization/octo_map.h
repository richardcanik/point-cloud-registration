#ifndef SRC_OCTO_MAP_H
#define SRC_OCTO_MAP_H

#include <localization/set.h>

class OctoMap {
public:
    explicit OctoMap(double leafSize);
    void fromSet(const Set &set);
    void getPoints(const std::vector<Condition*> &conditions, const double &distanceThreshold, std::vector<Point*> &points);

private:
    void point2VoxelIndex(const Point &point, long &index);
    void point2Coordinate(const Point &point, VoxelCoordinate &coordinate);
    void coordinate2VoxelIndex(const VoxelCoordinate &coordinate, long &index) const;
    long width;
    long height;
    long depth;
    double leafSize;
    Point offset;
    std::vector<std::vector<Point*>> data;
    VoxelCoordinate coordinateHelper;
};

#endif //SRC_OCTO_MAP_H
