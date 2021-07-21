#include <ros/ros.h>
#include <localization/pointCloud.h>


int main(int argc, char* argv[]) {
    ros::init(argc, argv, "vision_node");
    ros::NodeHandle nh;
    PointCloud destinationPointCloud(nh, "destination");
    ros::spin();
}
