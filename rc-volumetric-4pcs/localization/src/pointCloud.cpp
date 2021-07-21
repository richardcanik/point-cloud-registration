#include "localization/pointCloud.h"

PointCloud::PointCloud(ros::NodeHandle& nodeHandle, const std::string& name) :
    pointCloud(new pcl::PointCloud<pcl::PointXYZ>),
    publisher(nodeHandle.advertise<sensor_msgs::PointCloud2>(name, 1)),
    trigger(nodeHandle.advertiseService(name + "/publish", &PointCloud::publish, this)),
    setter(nodeHandle.advertiseService(name + "/set", &PointCloud::load, this)) {}

bool PointCloud::load(localization_msgs::String::Request &req, localization_msgs::String::Response &res) {
    this->reader.read("/upload/" + req.data, *this->pointCloud);
    return true;
}

bool PointCloud::publish(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*this->pointCloud, output);
    output.header.frame_id = "/base_link";
    this->publisher.publish(output);
    return true;
}
