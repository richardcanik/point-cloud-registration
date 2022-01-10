#include <ros/ros.h>
#include <registration_ros_wrapper/registration_api.h>

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "registration_node");
    ros::NodeHandle nh;
    RegistrationApi algorithm(nh);
    ros::spin();
}
