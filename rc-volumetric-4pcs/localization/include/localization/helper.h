#ifndef SRC_HELPER_H
#define SRC_HELPER_H

#include <pcl/io/ply_io.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

typedef Eigen::Vector3f Vector;
typedef Eigen::Transform<double, 3, Eigen::Affine> Transform;
typedef pcl::PointXYZ Point;
typedef pcl::PointCloud<Point> PointCloud;

struct VoxelCoordinate {
    long x = 0;
    long y = 0;
    long z = 0;
};

struct Condition {
    const Point *point = nullptr;
    const double *descriptor = nullptr;
};

enum POINT {
    P2,
    P3,
    P4
};

enum QUADRANT {
    ONE,
    TWO,
    THREE,
    FOUR
};

void getPointCloudFromMeshView(const pcl::PolygonMesh &mesh, PointCloud::Ptr &pointCloud);
double getLineLength(const Point &a, const Point &b);
bool checkSameNum(const size_t &i1, const size_t &i2, const size_t &i3, const size_t &i4);
void filterPointCloud(const PointCloud::Ptr &inputPointCloud, PointCloud::Ptr &outputPointCloud,
                      int numberOfPoints = 1000000);
void sphereParametricEquation(const Point &center, const double &radius, const double &s, const double &t,
                              Point &point);
void lineParametricEquation(const Point &p1, const Point &p2, const float &t, Point &point);
void circleParametricEquation(const Point &center, const double &radius, const float &t, const Vector &a,
                              const Vector &b, Point &point);

void publishPoints(const std::vector<Point> &p, ros::Publisher &publisher, const float &r = 1, const float &g = 0,
                   const float &b = 0, const float &size = 1);


#endif //SRC_HELPER_H
