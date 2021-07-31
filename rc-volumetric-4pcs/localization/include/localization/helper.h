#ifndef SRC_HELPER_H
#define SRC_HELPER_H

#include <pcl/io/ply_io.h>

typedef Eigen::Transform<double, 3, Eigen::Affine> TransformMatrix;

void getPointCloudFromMeshView(const pcl::PolygonMesh &mesh, pcl::PointCloud<pcl::PointXYZ>::Ptr &pointCloud);
double getLineLength(const pcl::PointXYZ &a, const pcl::PointXYZ &b);
bool checkSameNum(const size_t &i1, const size_t &i2, const size_t &i3, const size_t &i4);

#endif //SRC_HELPER_H
