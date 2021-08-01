#ifndef SRC_HELPER_H
#define SRC_HELPER_H

#include <pcl/io/ply_io.h>

typedef Eigen::Transform<double, 3, Eigen::Affine> TransformMatrix;
typedef pcl::PointXYZ Point;

void getPointCloudFromMeshView(const pcl::PolygonMesh &mesh, pcl::PointCloud<Point>::Ptr &pointCloud);
double getLineLength(const Point &a, const Point &b);
bool checkSameNum(const size_t &i1, const size_t &i2, const size_t &i3, const size_t &i4);
void filterPointCloud(const pcl::PointCloud<Point>::Ptr &inputPointCloud, pcl::PointCloud<Point>::Ptr &outputPointCloud,
                      int numberOfPoints = 1000000);

#endif //SRC_HELPER_H
