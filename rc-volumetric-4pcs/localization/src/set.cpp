#include "localization/set.h"

Set::Set(ros::NodeHandle &nodeHandle, const std::string &name) :
    publisherPointCloud(nodeHandle.advertise<sensor_msgs::PointCloud2>(name + "/point_cloud", 1)),
    publisherMesh(nodeHandle.advertise<visualization_msgs::Marker>(name + "/mesh", 1)),
    setPointCloud(nodeHandle.advertiseService(name + "/set/point_cloud", &Set::loadPointCloud, this)),
    setMesh(nodeHandle.advertiseService(name + "/set/mesh", &Set::loadMesh, this)),
    trigger(nodeHandle.advertiseService(name + "/publish", &Set::publish, this)),
    pointCloud(new pcl::PointCloud<Point>),
    isMesh(false),
    modelName("empty"),
    width(0),
    height(0),
    minBoundingBox{FLT_MAX, FLT_MAX, FLT_MAX},
    maxBoundingBox{FLT_MIN, FLT_MIN, FLT_MIN} {}

const pcl::PointCloud<Point>::Ptr &Set::getPointCloud() {
    return this->pointCloud;
}

const double &Set::getWidth() {
    return this->width;
}

const double &Set::getHeight() {
    return this->height;
}

bool Set::loadPointCloud(localization_msgs::String::Request &req, localization_msgs::String::Response &res) {
    pcl::PLYReader reader;
    if (reader.read("/upload/" + req.data, *this->pointCloud) == 0) {
        computeBoundingBox();
        res.success = true;
        this->modelName = req.data;
        this->isMesh = false;
    }
    return true;
}

bool Set::loadMesh(localization_msgs::String::Request &req, localization_msgs::String::Response &res) {
    if (pcl::io::loadPolygonFileSTL("/upload/" + req.data, this->mesh)) {
        getPointCloudFromMeshView(mesh, pointCloud);
        computeBoundingBox();
        res.success = true;
        this->modelName = req.data;
        this->isMesh = true;
    }
    return true;
}

bool Set::publish(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
    // Point Cloud to ROS
    sensor_msgs::PointCloud2 outputPointCloud;
    pcl::toROSMsg(*this->pointCloud, outputPointCloud);
    outputPointCloud.header.frame_id = "/base_link";
    this->publisherPointCloud.publish(outputPointCloud);

    // Mesh to ROS
    if (this->isMesh) {
        visualization_msgs::Marker outputMesh;
        outputMesh.header.frame_id = "base_link";
        outputMesh.header.stamp = ros::Time::now();
        outputMesh.ns = "my_namespace";
        outputMesh.id = 0;
        outputMesh.type = visualization_msgs::Marker::MESH_RESOURCE;
        outputMesh.action = visualization_msgs::Marker::ADD;
        outputMesh.pose.orientation.w = 1.0;
        outputMesh.color.a = 0.3;
        outputMesh.color.r = 1.0;
        outputMesh.mesh_resource = "package://model/" + this->modelName;
        this->publisherMesh.publish(outputMesh);
    }
    res.success = true;
    return true;
}

void Set::computeBoundingBox() {
    for (auto & point : this->pointCloud->points) {
        if (point.x < this->minBoundingBox.x) {
            this->minBoundingBox.x = point.x;
        }
        if (point.x > this->maxBoundingBox.x) {
            this->maxBoundingBox.x = point.x;
        }
        if (point.y < this->minBoundingBox.y) {
            this->minBoundingBox.y = point.y;
        }
        if (point.y > this->maxBoundingBox.y) {
            this->maxBoundingBox.y = point.y;
        }
        if (point.z < this->minBoundingBox.z) {
            this->minBoundingBox.z = point.z;
        }
        if (point.z > this->maxBoundingBox.z) {
            this->maxBoundingBox.z = point.z;
        }
    }
    this->width = this->maxBoundingBox.x - this->minBoundingBox.x;
    this->height = this->maxBoundingBox.y - this->minBoundingBox.y;
}
