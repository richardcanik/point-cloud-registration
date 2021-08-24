#ifndef SRC_OCTO_MAP_H
#define SRC_OCTO_MAP_H

#include <localization/set.h>

class OctoMap {
public:
    explicit OctoMap(ros::NodeHandle &nodeHandle, double leafSize);
    void fromSet(const Set &set);
    void getPoints(const std::vector<Condition*> &conditions, const double &distanceThreshold,
                   std::vector<Point*> &points);

private:
    void getPointsFromSphereSurface(const Point &center, const double &radius, const double &distanceThreshold,
                                    std::vector<Point*> &points);
    void getPointsFrom2SpheresIntersection(const std::vector<Condition*> &conditions, const double &distanceThreshold,
                                           std::vector<Point*> &points);
    void checkVoxel(const Point &point, const double &length, const long &index, const double &distanceThreshold,
                    std::vector<Point*> &points);
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
    std::vector<std::vector<Point*>> data;
    VoxelCoordinate coordinateHelper;

    ros::Publisher publisherPoints1;
    ros::Publisher publisherPoints2;
    ros::Publisher publisherPoints3;
    ros::Publisher publisherPoints4;
};

#endif //SRC_OCTO_MAP_H
