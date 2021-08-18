#include <localization/rc4pcs.h>

Rc4pcs::Rc4pcs(ros::NodeHandle &nodeHandle, const std::string &name) :
        alignService(nodeHandle.advertiseService(name + "/align", &Rc4pcs::align, this)),
        publisherBaseB(nodeHandle.advertise<visualization_msgs::Marker>(name + "/base_b", 1)),
        publisherDebug(nodeHandle.advertise<visualization_msgs::Marker>(name + "/debug", 1)),
        setP(nodeHandle, name + "/source"),
        setQ(nodeHandle, name + "/destination"),
        octoMapQ(3.0),
        distanceThreshold(0.3) {}

bool Rc4pcs::align(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
    Timer debugTimer;
    ROS_INFO("Align source %s to destination %s", this->setP.getModelName().c_str(), this->setQ.getModelName().c_str());
    this->alignTimer.start();

    this->selectBaseB();
    for (double i : this->baseB.getDescriptors()) {
        ROS_INFO("Descriptor: %f", i);
    }

    this->octoMapQ.fromSet(this->setQ);

    debugTimer.start();

    std::vector<std::vector<Point*>> candidates(3);
    std::vector<std::vector<Condition*>> conditions(3);

    Condition conditionP1P2{nullptr, &this->baseB.getDescriptors()[0]};
    Condition conditionP1P3{nullptr, &this->baseB.getDescriptors()[1]};
    Condition conditionP1P4{nullptr, &this->baseB.getDescriptors()[2]};
    Condition conditionP2P3{nullptr, &this->baseB.getDescriptors()[3]};
    Condition conditionP2P4{nullptr, &this->baseB.getDescriptors()[4]};
    Condition conditionP3P4{nullptr, &this->baseB.getDescriptors()[5]};

    conditions[0].push_back(&conditionP1P2);
    conditions[1].push_back(&conditionP1P3);
    conditions[1].push_back(&conditionP2P3);
    conditions[2].push_back(&conditionP1P4);
    conditions[2].push_back(&conditionP2P4);
    conditions[2].push_back(&conditionP3P4);

    Point *p1 = &this->setQ.getPointCloud()->points[0];
    conditionP1P2.point = p1;
    conditionP1P3.point = p1;
    conditionP1P4.point = p1;
    this->octoMapQ.getPoints(conditions[0], this->distanceThreshold, candidates[0]);
    for (auto &p2 : candidates[0]) {
        conditionP2P3.point = p2;
        conditionP2P4.point = p2;
        this->octoMapQ.getPoints(conditions[1], this->distanceThreshold, candidates[1]);
        for (auto &p3 : candidates[1]) {
            conditionP3P4.point = p3;
            this->octoMapQ.getPoints(conditions[2], this->distanceThreshold, candidates[2]);
            for (auto &p4 : candidates[2]) {
                ROS_INFO("Descriptor: %f", getLineLength(*p1, *p2));
                ROS_INFO("Descriptor: %f", getLineLength(*p1, *p3));
                ROS_INFO("Descriptor: %f", getLineLength(*p1, *p4));
                ROS_INFO("Descriptor: %f", getLineLength(*p2, *p3));
                ROS_INFO("Descriptor: %f", getLineLength(*p2, *p4));
                ROS_INFO("Descriptor: %f\n", getLineLength(*p3, *p4));
            }
        }
    }

    debugTimer.stop();
    this->alignTimer.stop();

    this->publishDebug(candidates[0]);
    this->publishBaseB();
    res.success = true;
    res.message = std::to_string(this->alignTimer.ms()) + "ms";

    ROS_INFO("Alignment took time %fms", this->alignTimer.ms());
    ROS_INFO("Get points took time %fms", debugTimer.ms());
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

void Rc4pcs::publishDebug(const std::vector<Point*> &p) {
    // Points
    visualization_msgs::Marker marker;
    geometry_msgs::Point point;

    marker.header.frame_id = "base_link";
    marker.header.stamp = ros::Time::now();
    marker.ns = "debug";
    marker.action = visualization_msgs::Marker::ADD;
    marker.type = visualization_msgs::Marker::POINTS;
    marker.id = 0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 1;
    marker.scale.y = 1;
    marker.scale.z = 1;
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    for (auto i : p) {
        point.x = i->x;
        point.y = i->y;
        point.z = i->z;
        marker.points.push_back(point);
    }
    this->publisherDebug.publish(marker);
}
