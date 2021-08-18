#ifndef SRC_SET_H
#define SRC_SET_H

#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/PolygonMesh.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_srvs/Trigger.h>
#include <visualization_msgs/Marker.h>
#include <pcl_conversions/pcl_conversions.h>
#include <localization/helper.h>
#include <localization_msgs/String.h>
#include <localization/timer.h>

class Set {
public:
    Set(ros::NodeHandle& nodeHandle, const std::string& name);
    const PointCloud::Ptr &getPointCloud() const;
    const double &getWidth() const;
    const double &getHeight() const;
    const double &getDepth() const;
    const std::string &getModelName() const;
    const Point &getMinBoundingBox() const;
    const Point &getMaxBoundingBox() const;

private:
    bool loadPointCloud(localization_msgs::String::Request &req, localization_msgs::String::Response &res);
    bool loadMesh(localization_msgs::String::Request &req, localization_msgs::String::Response &res);
    bool publish(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
    void computeBoundingBox();

    ros::Publisher publisherPointCloud;
    ros::Publisher publisherMesh;
    ros::Publisher publisherBoundingBox;
    ros::ServiceServer setPointCloud;
    ros::ServiceServer setMesh;
    ros::ServiceServer trigger;
    PointCloud::Ptr pointCloud;
    pcl::PolygonMesh mesh;
    bool isMesh;
    std::string modelName;
    double width;
    double height;
    double depth;
    Point minBoundingBox;
    Point maxBoundingBox;
};

#endif //SRC_SET_H
