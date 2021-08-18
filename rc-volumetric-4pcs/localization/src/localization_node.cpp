#include <ros/ros.h>
#include <localization/rc4pcs.h>

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "localization_node");
    ros::NodeHandle nh;
    Rc4pcs algorithm(nh, "localization");
    ros::spin();
}
