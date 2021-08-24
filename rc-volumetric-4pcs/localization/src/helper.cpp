#include <localization/helper.h>

void getPointCloudFromMeshView(const pcl::PolygonMesh &mesh, PointCloud::Ptr &pointCloud) {
    PointCloud cloud;
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

double getLineLength(const Point &a, const Point &b) {
    return sqrt(pow(a.x-b.x, 2) + pow(a.y-b.y, 2) + pow(a.z-b.z, 2));
}

bool checkSameNum(const size_t &i1, const size_t &i2, const size_t &i3, const size_t &i4) {
    return !(i1 == i2 || i1 == i3 || i1 == i4 || i2 == i3 || i2 == i4 || i3 == i4);
}

void filterPointCloud(const PointCloud::Ptr &inputPointCloud, PointCloud::Ptr &outputPointCloud, int numberOfPoints) {
    *outputPointCloud = *inputPointCloud;
    if (inputPointCloud->points.size() > numberOfPoints) {
        outputPointCloud->points.clear();
        for (size_t i = 0;
             i < inputPointCloud->points.size(); i += size_t(inputPointCloud->points.size() / numberOfPoints)) {
            outputPointCloud->points.push_back(inputPointCloud->points[i]);
        }
        outputPointCloud->width = 1;
        outputPointCloud->height = outputPointCloud->points.size();
    }
}

void sphereParametricEquation(const Point &center, const double &radius, const double &s, const double &t,
                              Point &point) {
    point = {center.x + static_cast<float>(radius * cos(s) * sin(t)),
             center.y + static_cast<float>(radius * sin(s) * sin(t)),
             center.z + static_cast<float>(radius * cos(t))};
}

void lineParametricEquation(const Point &p1, const Point &p2, const float &t, Point &point) {
    point = {p1.x + (t * (p2.x - p1.x)),
             p1.y + (t * (p2.y - p1.y)),
             p1.z + (t * (p2.z - p1.z))};
}

void circleParametricEquation(const Point &center, const double &radius, const double &t, const Vector &a,
                              const Vector &b, Point &point) {
    Vector v1 = a.cross(b);
    Vector v2 = a.normalized();
    v1 = v1.cross(v2).normalized();
    point = {center.x + static_cast<float>(radius * cos(t) * v1.x() + radius * sin(t) * v2.x()),
             center.y + static_cast<float>(radius * cos(t) * v1.y() + radius * sin(t) * v2.y()),
             center.z + static_cast<float>(radius * cos(t) * v1.z() + radius * sin(t) * v2.z())};
}

bool isTriangle(const double &a, const double &b, const double &c) {
    return (a + b > c && a + c > b && b + c > a);
}

void publishPoints(const std::vector<Point> &p, ros::Publisher &publisher, const float &r, const float &g,
                   const float &b, const float &size) {
    // Points
    visualization_msgs::Marker marker;
    geometry_msgs::Point point;

    marker.header.frame_id = "base_link";
    marker.header.stamp = ros::Time::now();
    marker.ns = "debug";
    marker.action = visualization_msgs::Marker::ADD;
    marker.type = visualization_msgs::Marker::POINTS;
    marker.id = 0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = size;
    marker.scale.y = size;
    marker.scale.z = size;
    marker.color.a = 1.0;
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    for (auto i : p) {
        point.x = i.x;
        point.y = i.y;
        point.z = i.z;
        marker.points.push_back(point);
    }
    publisher.publish(marker);
}
