#ifndef SRC_HELPER_H
#define SRC_HELPER_H

#include <pcl/io/ply_io.h>
#include <pcl/octree/octree_search.h>

struct MatrixCoordinate {
    long x = 0;
    long y = 0;
};
typedef Eigen::Transform<double, 3, Eigen::Affine> TransformMatrix;
typedef pcl::PointXYZ Point;
typedef pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> OcTree;
typedef std::vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ>> OcMap;

void getPointCloudFromMeshView(const pcl::PolygonMesh &mesh, pcl::PointCloud<Point>::Ptr &pointCloud);
double getLineLength(const Point &a, const Point &b);
bool checkSameNum(const size_t &i1, const size_t &i2, const size_t &i3, const size_t &i4);
void filterPointCloud(const pcl::PointCloud<Point>::Ptr &inputPointCloud, pcl::PointCloud<Point>::Ptr &outputPointCloud,
                      int numberOfPoints = 1000000);

#endif //SRC_HELPER_H
