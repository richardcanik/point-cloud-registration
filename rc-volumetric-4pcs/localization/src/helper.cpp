#include <localization/helper.h>

void getPointCloudFromMeshView(const pcl::PolygonMesh &mesh, pcl::PointCloud<pcl::PointXYZ>::Ptr &pointCloud) {
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromPCLPointCloud2(mesh.cloud, cloud);
    pointCloud->points.clear();
    for(auto currentPoly : mesh.polygons) {
        // TODO pass view as argument
        Eigen::Vector3f view{1, 0, 0};
        Eigen::Vector3f vec12(cloud[currentPoly.vertices[1]].x-cloud[currentPoly.vertices[0]].x,
                              cloud[currentPoly.vertices[1]].y-cloud[currentPoly.vertices[0]].y,
                              cloud[currentPoly.vertices[1]].z-cloud[currentPoly.vertices[0]].z);
        Eigen::Vector3f vec23(cloud[currentPoly.vertices[2]].x-cloud[currentPoly.vertices[1]].x,
                              cloud[currentPoly.vertices[2]].y-cloud[currentPoly.vertices[1]].y,
                              cloud[currentPoly.vertices[2]].z-cloud[currentPoly.vertices[1]].z);
        if (fabs(acos(vec12.cross(vec23).normalized().dot(view.normalized()))) < (M_PI / 3)) {
            for (unsigned int vertice : currentPoly.vertices) {
                // TODO find a way on how to define the cutting plane
                if (cloud[vertice].x > 0)
                    pointCloud->points.push_back(cloud[vertice]);
            }
        }
    }
}

double getLineLength(const pcl::PointXYZ &a, const pcl::PointXYZ &b) {
    return sqrt(pow(a.x-b.x, 2) + pow(a.y-b.y, 2) + pow(a.z-b.z, 2));
}

bool checkSameNum(const size_t &i1, const size_t &i2, const size_t &i3, const size_t &i4) {
    return !(i1 == i2 || i1 == i3 || i1 == i4 || i2 == i3 || i2 == i4 || i3 == i4);
}
