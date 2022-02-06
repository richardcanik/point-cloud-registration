#ifndef SRC_REGISTRATION_API_H
#define SRC_REGISTRATION_API_H

#include <registration_core/registration.h>
#include <registration_ros_wrapper/helper.h>
#include <ros/ros.h>
#include <pcl/common/transforms.h>
#include <std_srvs/Trigger.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <registration_msgs/String.h>

class RegistrationApi : Registration {
public:
    explicit RegistrationApi(ros::NodeHandle &nh, const std::string &name = "/registration_api");

private:
    bool setSetPServiceFunction(registration_msgs::String::Request &req, registration_msgs::String::Response &res);
    bool setSetQServiceFunction(registration_msgs::String::Request &req, registration_msgs::String::Response &res);
    bool alignServiceFunction(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
    bool sourcePublishServiceFunction(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
    bool destinationPublishServiceFunction(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
    void sourcePublishTrigger();
    void destinationPublishTrigger();

    ros::ServiceServer setSetPService;
    ros::ServiceServer setSetQService;
    ros::ServiceServer alignService;
    ros::ServiceServer sourceViewerPublishService;
    ros::ServiceServer destinationViewerPublishService;
    ros::Publisher setPPublisher;
    ros::Publisher setPTransformedPublisher;
    ros::Publisher setQPublisher;
    ros::Publisher setPBoundingBoxPublisher;
    ros::Publisher setQBoundingBoxPublisher;
    ros::Publisher baseBPublisher;
    ros::Publisher baseBTransformedPublisher;
    ros::Publisher baseUPublisher;
};

#endif //SRC_REGISTRATION_API_H
