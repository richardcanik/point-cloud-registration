#include <ros/ros.h>
#include <registration/rc4pcs.h>

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "registration_node");
    ros::NodeHandle nh;
    Rc4pcs algorithm(nh, "registration");
    ros::spin();
}
