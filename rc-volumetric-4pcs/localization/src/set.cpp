#include "localization/set.h"

Set::Set(ros::NodeHandle &nodeHandle, const std::string &name) :
    publisherPointCloud(nodeHandle.advertise<sensor_msgs::PointCloud2>(name + "/point_cloud", 1)),
    publisherMesh(nodeHandle.advertise<visualization_msgs::Marker>(name + "/mesh", 1)),
    publisherBoundingBox(nodeHandle.advertise<visualization_msgs::Marker>(name + "/bounding_box", 1)),
    setPointCloud(nodeHandle.advertiseService(name + "/set/point_cloud", &Set::loadPointCloud, this)),
    setMesh(nodeHandle.advertiseService(name + "/set/mesh", &Set::loadMesh, this)),
    trigger(nodeHandle.advertiseService(name + "/publish", &Set::publish, this)),
    pointCloud(new pcl::PointCloud<Point>),
    isMesh(false),
    modelName("empty"),
    width(0),
    height(0) {}

const pcl::PointCloud<Point>::Ptr &Set::getPointCloud() {
    return this->pointCloud;
}

const double &Set::getWidth() {
    return this->width;
}

const double &Set::getHeight() {
    return this->height;
}

const std::string &Set::getModelName() {
    return this->modelName;
}

const Point &Set::getMinBoundingBox() {
    return this->minBoundingBox;
}

const Point &Set::getMaxBoundingBox() {
    return this->maxBoundingBox;
}

bool Set::loadPointCloud(localization_msgs::String::Request &req, localization_msgs::String::Response &res) {
    auto start = std::chrono::system_clock::now();
    pcl::PLYReader reader;
    if (reader.read("/upload/" + req.data, *this->pointCloud) == 0) {
        computeBoundingBox();
        res.success = true;
        this->modelName = req.data;
        this->isMesh = false;
    }
    auto end = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsedSeconds = end - start;
    ROS_INFO("Load Point Cloud took %fms", elapsedSeconds.count() * 1000);
    return true;
}

bool Set::loadMesh(localization_msgs::String::Request &req, localization_msgs::String::Response &res) {
    auto start = std::chrono::system_clock::now();
    if (pcl::io::loadPolygonFileSTL("/upload/" + req.data, this->mesh)) {
        getPointCloudFromMeshView(mesh, pointCloud);
        computeBoundingBox();
        res.success = true;
        this->modelName = req.data;
        this->isMesh = true;
    }
    auto end = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsedSeconds = end - start;
    ROS_INFO("Load Mesh took %fms", elapsedSeconds.count() * 1000);
    return true;
}

bool Set::publish(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
    // Point Cloud
    sensor_msgs::PointCloud2 outputPointCloud;
    pcl::PointCloud<Point>::Ptr filteredPointCloud(new pcl::PointCloud<Point>);
    filterPointCloud(this->pointCloud, filteredPointCloud);
    pcl::toROSMsg(*filteredPointCloud, outputPointCloud);
    outputPointCloud.header.frame_id = "/base_link";
    this->publisherPointCloud.publish(outputPointCloud);

    // Mesh
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

    // Bounding Box
    visualization_msgs::Marker outputBoundingBox;
    std::vector<int> index{0, 1, 3, 2, 0, 4, 5, 7, 6, 4, 0, 1, 5, 7, 3, 2, 6};
    std::vector<geometry_msgs::Point> p(8);
    p[0].x = this->minBoundingBox.x;
    p[0].y = this->minBoundingBox.y;
    p[0].z = this->minBoundingBox.z;
    p[1].x = this->minBoundingBox.x;
    p[1].y = this->minBoundingBox.y;
    p[1].z = this->maxBoundingBox.z;
    p[2].x = this->minBoundingBox.x;
    p[2].y = this->maxBoundingBox.y;
    p[2].z = this->minBoundingBox.z;
    p[3].x = this->minBoundingBox.x;
    p[3].y = this->maxBoundingBox.y;
    p[3].z = this->maxBoundingBox.z;
    p[4].x = this->maxBoundingBox.x;
    p[4].y = this->minBoundingBox.y;
    p[4].z = this->minBoundingBox.z;
    p[5].x = this->maxBoundingBox.x;
    p[5].y = this->minBoundingBox.y;
    p[5].z = this->maxBoundingBox.z;
    p[6].x = this->maxBoundingBox.x;
    p[6].y = this->maxBoundingBox.y;
    p[6].z = this->minBoundingBox.z;
    p[7].x = this->maxBoundingBox.x;
    p[7].y = this->maxBoundingBox.y;
    p[7].z = this->maxBoundingBox.z;
    outputBoundingBox.header.frame_id = "base_link";
    outputBoundingBox.header.stamp = ros::Time::now();
    outputBoundingBox.ns = "base";
    outputBoundingBox.action = visualization_msgs::Marker::ADD;
    outputBoundingBox.pose.orientation.w = 1.0;
    outputBoundingBox.id = 0;
    outputBoundingBox.type = visualization_msgs::Marker::LINE_STRIP;
    outputBoundingBox.color.r = 1.0;
    outputBoundingBox.color.a = 1.0;
    for (int i : index) {
        outputBoundingBox.points.push_back(p[i]);
    }
    this->publisherBoundingBox.publish(outputBoundingBox);

    res.success = true;
    return true;
}

void Set::computeBoundingBox() {
    this->minBoundingBox = {FLT_MAX, FLT_MAX, FLT_MAX};
    this->maxBoundingBox = {FLT_MIN, FLT_MIN, FLT_MIN};
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
