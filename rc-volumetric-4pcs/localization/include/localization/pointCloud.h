#ifndef SRC_POINTCLOUD_H
#define SRC_POINTCLOUD_H

#include <pcl/io/ply_io.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_srvs/Trigger.h>
#include <localization_msgs/String.h>
#include <pcl_conversions/pcl_conversions.h>

class PointCloud {
public:
    PointCloud(ros::NodeHandle& nodeHandle, const std::string& name);
    bool publish(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
    bool load(localization_msgs::String::Request &req, localization_msgs::String::Response &res);

private:
    pcl::PLYReader reader;
    ros::Publisher publisher;
    ros::ServiceServer trigger;
    ros::ServiceServer setter;
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud;
};

#endif //SRC_POINTCLOUD_H
