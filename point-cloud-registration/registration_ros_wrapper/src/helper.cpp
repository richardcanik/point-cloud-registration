#include <registration_ros_wrapper/helper.h>
#include <pcl/point_types.h>

bool setFromPly(const std::string &path, Set &set) {
    pcl::PLYReader reader;
    PointCloud::Ptr pointCloud(new PointCloud);
    std::vector<Point> _set;
    if (reader.read(path, *pointCloud) == 0) {
        _set.assign(pointCloud->points.begin(), pointCloud->points.end());
        set.setSet(_set);
        return true;
    }
    return false;
}

void setToPointCloud(const Set &set, PointCloud::Ptr &pointCloud) {
    pointCloud->points.clear();
    pointCloud->points.assign(set.getSet().begin(), set.getSet().end());
}

void setToPointCloud(const Set &set, sensor_msgs::PointCloud2 &pointCloud2) {
    PointCloud::Ptr pointCloud(new PointCloud);
    setToPointCloud(set, pointCloud);
    pointCloudToPointCloud2(pointCloud, pointCloud2);
}

void pointCloudToPointCloud2(const PointCloud::Ptr &pointCloud, sensor_msgs::PointCloud2 &pointCloud2) {
    PointCloud::Ptr filteredPointCloud(new PointCloud);
    filterPointCloud(pointCloud, filteredPointCloud);
    pcl::toROSMsg(*filteredPointCloud, pointCloud2);
    pointCloud2.header.frame_id = "/base_link";
    pointCloud2.header.stamp = ros::Time::now();
}

void filterPointCloud(const PointCloud::Ptr &inputPointCloud, PointCloud::Ptr &outputPointCloud, int numberOfPoints) {
    *outputPointCloud = *inputPointCloud;
    if (inputPointCloud->points.size() > numberOfPoints) {
        outputPointCloud->points.clear();
        for (size_t i = 0; i < inputPointCloud->points.size(); i += size_t(inputPointCloud->points.size() / numberOfPoints)) {
            outputPointCloud->points.push_back(inputPointCloud->points[i]);
        }
        outputPointCloud->width = 1;
        outputPointCloud->height = outputPointCloud->points.size();
    }
}

void toGeometryPoint(geometry_msgs::Point &point, const double &x, const double &y, const double &z) {
    point.x = x;
    point.y = y;
    point.z = z;
}

void boundingBoxToMarker(const Point &min, const Point &max, visualization_msgs::Marker &marker) {
    std::vector<int> index{0, 1, 3, 2, 0, 4, 5, 7, 6, 4, 0, 1, 5, 7, 3, 2, 6};
    std::vector<geometry_msgs::Point> p(8);
    toGeometryPoint(p[0], min.x, min.y, min.z);
    toGeometryPoint(p[1], min.x, min.y, max.z);
    toGeometryPoint(p[2], min.x, max.y, min.z);
    toGeometryPoint(p[3], min.x, max.y, max.z);
    toGeometryPoint(p[4], max.x, min.y, min.z);
    toGeometryPoint(p[5], max.x, min.y, max.z);
    toGeometryPoint(p[6], max.x, max.y, min.z);
    toGeometryPoint(p[7], max.x, max.y, max.z);
    marker.header.frame_id = "base_link";
    marker.header.stamp = ros::Time::now();
    marker.ns = "base";
    marker.action = visualization_msgs::Marker::ADD;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.pose.orientation.w = 1.0;
    marker.id = 0;
    marker.color.r = 1.0;
    marker.color.a = 1.0;
    for (int i : index) {
        marker.points.push_back(p[i]);
    }
}

void baseToMarker(const Base &base, visualization_msgs::Marker &marker, const int &color, const Transform &transform) {
    std::vector<int> index{0, 1, 2, 3, 0, 2, 1, 3};
    geometry_msgs::Point p;
    Point point;
    marker.header.frame_id = "base_link";
    marker.header.stamp = ros::Time::now();
    marker.ns = "base";
    marker.action = visualization_msgs::Marker::ADD;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.pose.orientation.w = 1.0;
    marker.id = 0;
    marker.color.r = static_cast<float>(((color >> 16) & 0xFF) / 255.0);
    marker.color.g = static_cast<float>(((color >> 8) & 0xFF) / 255.0);
    marker.color.b = static_cast<float>(((color) & 0xFF) / 255.0);
    marker.color.a = 1.0;
    for (int i : index) {
        point = base.getPoints()[i];
        transformPoint(point, transform);
        toGeometryPoint(p, point.x, point.y, point.z);
        marker.points.push_back(p);
    }
}
