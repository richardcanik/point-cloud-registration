#ifndef SRC_HELPER_H
#define SRC_HELPER_H

#include <pcl/io/ply_io.h>

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

void getPointCloudFromMeshView(const pcl::PolygonMesh &mesh, PointCloud::Ptr &pointCloud);
double getLineLength(const Point &a, const Point &b);
bool checkSameNum(const size_t &i1, const size_t &i2, const size_t &i3, const size_t &i4);
void filterPointCloud(const PointCloud::Ptr &inputPointCloud, PointCloud::Ptr &outputPointCloud, int numberOfPoints = 1000000);

#endif //SRC_HELPER_H
