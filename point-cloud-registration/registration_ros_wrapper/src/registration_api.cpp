#include <registration_ros_wrapper/registration_api.h>

RegistrationApi::RegistrationApi(ros::NodeHandle &nh, const std::string &name) :
        setSetPService(nh.advertiseService(name + "/source/set", &RegistrationApi::setSetPServiceFunction, this)),
        setSetQService(nh.advertiseService(name + "/destination/set", &RegistrationApi::setSetQServiceFunction, this)),
        alignService(nh.advertiseService(name + "/align", &RegistrationApi::alignServiceFunction, this)),
        sourceViewerPublishService(nh.advertiseService(name + "/source/publish", &RegistrationApi::sourcePublishServiceFunction, this)),
        destinationViewerPublishService(nh.advertiseService(name + "/destination/publish", &RegistrationApi::destinationPublishServiceFunction, this)),
        setPPublisher(nh.advertise<sensor_msgs::PointCloud2>(name + "/source/point_cloud", 1)),
        setPTransformedPublisher(nh.advertise<sensor_msgs::PointCloud2>(name + "/source/point_cloud/transformed", 1)),
        setQPublisher(nh.advertise<sensor_msgs::PointCloud2>(name + "/destination/point_cloud", 1)),
        setPBoundingBoxPublisher(nh.advertise<visualization_msgs::Marker>(name + "/source/bounding_box", 1)),
        setQBoundingBoxPublisher(nh.advertise<visualization_msgs::Marker>(name + "/destination/bounding_box", 1)),
        baseBPublisher(nh.advertise<visualization_msgs::Marker>(name + "/base_b", 1)),
        baseBTransformedPublisher(nh.advertise<visualization_msgs::Marker>(name + "/base_b/transformed", 1)),
        baseUPublisher(nh.advertise<visualization_msgs::Marker>(name + "/base_u", 1)) {}

bool RegistrationApi::setSetPServiceFunction(registration_msgs::String::Request &req, registration_msgs::String::Response &res) {
    if (setFromPly("/upload/" + req.data, this->setP)) {
        this->sourcePublishTrigger();
        res.success = true;
        return true;
    }
    res.success = false;
    return false;
}

bool RegistrationApi::setSetQServiceFunction(registration_msgs::String::Request &req, registration_msgs::String::Response &res) {
    if (setFromPly("/upload/" + req.data, this->setQ)) {
        this->destinationPublishTrigger();
        res.success = true;
        return true;
    }
    res.success = false;
    return false;
}

bool RegistrationApi::alignServiceFunction(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
    // Alignment
    this->align();

    // Publish Base B
    visualization_msgs::Marker baseB;
    baseToMarker(this->getBaseB(), baseB);
    baseBPublisher.publish(baseB);

    // Results
    PointCloud::Ptr pointCloudPTransformed(new PointCloud);
    sensor_msgs::PointCloud2 pointCloud2PTransformed;
    visualization_msgs::Marker baseU, baseBTransformed;
    std::cout << "Number of results: " << this->getResults().size() << std::endl;
    for (auto &result : this->getResults()) {
        std::cout << "Alignment Time[ms]: " << result.alignmentTime << ", Overlap[%]: " << result.overlap << std::endl;

        // Publish Base U
        baseToMarker(result.baseU, baseU, 0x008000);
        baseUPublisher.publish(baseU);  // TODO publish each result at separate topic

        // Publish Transformed Set P
        setToPointCloud(this->setP, pointCloudPTransformed);
        pcl::transformPointCloud(*pointCloudPTransformed, *pointCloudPTransformed, result.transform);
        pointCloudToPointCloud2(pointCloudPTransformed, pointCloud2PTransformed);
        setPTransformedPublisher.publish(pointCloud2PTransformed);

        // Publish Transformed Base B
        baseToMarker(this->getBaseB(), baseBTransformed, 0x0044cc, result.transform);
        baseBTransformedPublisher.publish(baseBTransformed);
    }

    res.success = true;
    return true;
}

bool RegistrationApi::sourcePublishServiceFunction(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
    this->sourcePublishTrigger();
    res.success = true;
    return true;
}

bool RegistrationApi::destinationPublishServiceFunction(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
    this->destinationPublishTrigger();
    res.success = true;
    return true;
}

void RegistrationApi::sourcePublishTrigger() {
    sensor_msgs::PointCloud2 pointCloud;
    visualization_msgs::Marker boundingBox;
    setToPointCloud(this->setP, pointCloud);
    boundingBoxToMarker(this->setP.getMinBoundingBox(), this->setP.getMaxBoundingBox(), boundingBox);
    this->setPPublisher.publish(pointCloud);
    this->setPBoundingBoxPublisher.publish(boundingBox);
}

void RegistrationApi::destinationPublishTrigger() {
    sensor_msgs::PointCloud2 pointCloud;
    visualization_msgs::Marker boundingBox;
    setToPointCloud(this->setQ, pointCloud);
    boundingBoxToMarker(this->setQ.getMinBoundingBox(), this->setQ.getMaxBoundingBox(), boundingBox);
    this->setQPublisher.publish(pointCloud);
    this->setQBoundingBoxPublisher.publish(boundingBox);
}
