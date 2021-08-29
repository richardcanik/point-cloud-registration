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
    void getPointsFrom2SpheresIntersection(const Point &center1, const double &radius1,
                                           const Point &center2, const double &radius2,
                                           const double &distanceThreshold, std::vector<Point*> &points);
    void getPointsFrom3SpheresIntersection(const Point &center1, const double &radius1,
                                           const Point &center2, const double &radius2,
                                           const Point &center3, const double &radius3,
                                           const double &distanceThreshold, std::vector<Point*> &points);
    void getCircleFrom2SphereIntersection(const Point &center1, const double &radius1,
                                          const Point &center2, const double &radius2,
                                          Point &center, double &radius, Vector &v1, Vector &v2, int &status);
    void getPointsFromCircleSphereIntersection(const Point &centerCircle, const double &radiusCircle,
                                               const Vector &v1, const Vector &v2,
                                               const Point &centerSphere, const double &radiusSphere,
                                               std::vector<Point> &points, int &status);
    void checkVoxel(const Point &point, const double &length, const long &index, const double &distanceThreshold,
                    std::vector<Point*> &points);
    void checkVoxel(const long &index, const double &distanceThreshold, std::vector<Point*> &points);
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
    const std::vector<Condition*> *conditions;

    ros::Publisher publisherPoints1;
    ros::Publisher publisherPoints2;
    ros::Publisher publisherPoints3;
    ros::Publisher publisherPoints4;
    ros::Publisher publisherPoints5;
    ros::Publisher publisherPoints6;
};

#endif //SRC_OCTO_MAP_H
