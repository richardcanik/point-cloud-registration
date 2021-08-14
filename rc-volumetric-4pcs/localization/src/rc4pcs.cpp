#include <localization/rc4pcs.h>

Rc4pcs::Rc4pcs(ros::NodeHandle &nodeHandle, const std::string &name) :
    alignService(nodeHandle.advertiseService(name + "/align", &Rc4pcs::align, this)),
    publisherBaseB(nodeHandle.advertise<visualization_msgs::Marker>(name + "/base_b", 1)),
    publisherDebug(nodeHandle.advertise<visualization_msgs::Marker>(name + "/debug", 1)),
    setP(nodeHandle, name + "/source"),
    setQ(nodeHandle, name + "/destination"),
    pointDistance(1.0),
    matrix(1.0) {}

bool Rc4pcs::align(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
    ROS_INFO("Align source %s to destination %s", this->setP.getModelName().c_str(), this->setQ.getModelName().c_str());
    auto start = std::chrono::system_clock::now();
    this->selectBaseB();
    this->matrix.fromSet(this->setQ);
    std::vector<Point*> candidate;
    this->matrix.getPoints(this->setQ.getPointCloud()->points[0], 1.0, candidate);
    auto end = std::chrono::system_clock::now();

    this->publishBaseB();
    std::chrono::duration<double> elapsedSeconds = end - start;
    ROS_INFO("Alignment took %fms", elapsedSeconds.count() * 1000);
    res.success = true;
    res.message = std::to_string(elapsedSeconds.count() * 1000) + "ms";
    return true;
}

void Rc4pcs::selectBaseB() {
    size_t i1, i2, i3, i4;
    Randomer random{0, this->setP.getPointCloud()->points.size() - 1};
    while (true) {
        if (checkSameNum(i1 = random(), i2 = random(), i3 = random(), i4 = random())) {
            if (this->baseB.setBase(this->setP.getPointCloud()->points[i1],
                                    this->setP.getPointCloud()->points[i2],
                                    this->setP.getPointCloud()->points[i3],
                                    this->setP.getPointCloud()->points[i4],
                                    0.5*std::max(this->setP.getWidth(), this->setP.getHeight()))) {
                break;
            }
        }
    }
}

void Rc4pcs::computePointDistance(double seed) {
    double distance = seed;
    for (size_t i = 0; i < 100; i++) {
        double length = getLineLength(this->setQ.getPointCloud()->points[i], this->setQ.getPointCloud()->points[i+1]);
        if (length < distance + 0.5 && length > distance - 0.5) {
            distance = (distance + length) / 2;
        }
    }
    this->pointDistance = distance;
}

void Rc4pcs::publishBaseB() {
    visualization_msgs::Marker lines;
    geometry_msgs::Point p;
    std::vector<int> index{0, 1, 2, 3, 0, 2, 1, 3};

    lines.header.frame_id = "base_link";
    lines.header.stamp = ros::Time::now();
    lines.ns = "base";
    lines.action = visualization_msgs::Marker::ADD;
    lines.type = visualization_msgs::Marker::LINE_STRIP;
    lines.pose.orientation.w = 1.0;
    lines.id = 0;
    lines.color.b = 1.0;
    lines.color.a = 1.0;
    for (int i : index) {
        p.x = this->baseB.getPoints()[i].x;
        p.y = this->baseB.getPoints()[i].y;
        p.z = this->baseB.getPoints()[i].z;
        lines.points.push_back(p);
    }
    publisherBaseB.publish(lines);
}

void Rc4pcs::publishDebug(const Point &p) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "base_link";
    marker.header.stamp = ros::Time::now();
    marker.ns = "debug";
    marker.action = visualization_msgs::Marker::ADD;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.id = 0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 10;
    marker.scale.y = 10;
    marker.scale.z = 10;
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.pose.position.x = p.x;
    marker.pose.position.y = p.y;
    marker.pose.position.z = p.z;
    this->publisherDebug.publish(marker);
}
