/**
**  Simple ROS Node
**/
#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <pcl/io/ply_io.h>
#include <pcl/PCLPointCloud2.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

pcl::PointCloud<pcl::PointXYZ>::Ptr destination(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PLYReader destinationReader;
ros::Publisher pointCloudPub;

bool point_cloud_publish(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
    sensor_msgs::PointCloud2 output;

    ROS_ERROR("Point Cloud upload");
    destinationReader.read("/upload/scene-T-fittings.ply", *destination);
    pcl::toROSMsg(*destination, output);
    output.header.frame_id = "/base_link";
    pointCloudPub.publish(output);
    return true;
}

int main(int argc, char* argv[]) {
    // This must be called before anything else ROS-related
    ros::init(argc, argv, "vision_node");

    // Create a ROS node handle
    ros::NodeHandle nh;

    // Create service server to trigger to publish point cloud
    ros::ServiceServer service = nh.advertiseService("point_cloud/publish", point_cloud_publish);
    ROS_INFO("Hello, World!");

    // Create publisher of point cloud
    pointCloudPub = nh.advertise<sensor_msgs::PointCloud2>("/destination_point_cloud", 1);

    // Don't exit the program.
    ros::spin();
}
