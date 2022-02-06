#ifndef SRC_API_HELPER_H
#define SRC_API_HELPER_H

#include <registration_core/math.h>
#include <registration_core/set.h>
#include <registration_core/base.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>

typedef pcl::PointXYZ PointCloudPoints;
typedef pcl::PointCloud<PointCloudPoints> PointCloud;

bool setFromPly(const std::string &path, Set &set);
void setToPointCloud(const Set &set, PointCloud::Ptr &pointCloud);
void setToPointCloud(const Set &set, sensor_msgs::PointCloud2 &pointCloud2);
void pointCloudToPointCloud2(const PointCloud::Ptr &pointCloud, sensor_msgs::PointCloud2 &pointCloud2);
void filterPointCloud(const PointCloud::Ptr &inputPointCloud, PointCloud::Ptr &outputPointCloud, int numberOfPoints = 100000);
void toGeometryPoint(geometry_msgs::Point &point, const double &x, const double &y, const double &z);
void boundingBoxToMarker(const Point &min, const Point &max, visualization_msgs::Marker &marker);
void baseToMarker(const Base &base, visualization_msgs::MarkerArray &marker, const int &color = 0x0044cc, const Transform &transform = Transform::Identity());
PointCloudPoints convertPointToPointCloudPoints(const Point &point);
Point convertPointCloudPointsToPoint(const PointCloudPoints &pointCloudPoints);

#endif //SRC_API_HELPER_H
