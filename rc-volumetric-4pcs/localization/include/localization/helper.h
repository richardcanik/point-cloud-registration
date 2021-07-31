#ifndef SRC_HELPER_H
#define SRC_HELPER_H

#include <pcl/io/ply_io.h>

typedef Eigen::Transform<double, 3, Eigen::Affine> TransformMatrix;
typedef pcl::PointXYZ Point;

void getPointCloudFromMeshView(const pcl::PolygonMesh &mesh, pcl::PointCloud<Point>::Ptr &pointCloud);
double getLineLength(const Point &a, const Point &b);
bool checkSameNum(const size_t &i1, const size_t &i2, const size_t &i3, const size_t &i4);

#endif //SRC_HELPER_H
