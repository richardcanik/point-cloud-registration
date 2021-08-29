#include <localization/rc4pcs.h>

Rc4pcs::Rc4pcs(ros::NodeHandle &nodeHandle, const std::string &name) :
        alignService(nodeHandle.advertiseService(name + "/align", &Rc4pcs::align, this)),
        publisherBaseB(nodeHandle.advertise<visualization_msgs::Marker>(name + "/base_b", 1)),
        publisherDebug(nodeHandle.advertise<visualization_msgs::Marker>(name + "/debug", 1)),
        setP(nodeHandle, name + "/source"),
        setQ(nodeHandle, name + "/destination"),
        octoMapQ(nodeHandle, 3.0),
        distanceThreshold(0.3) {}

bool Rc4pcs::align(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
    ROS_INFO("Align source %s to destination %s", this->setP.getModelName().c_str(), this->setQ.getModelName().c_str());
    this->alignTimer.start();
    this->selectBaseB();
    this->octoMapQ.fromSet(this->setQ);
    this->findCandidate(this->setQ.getPointCloud()->points[0]);
    this->alignTimer.stop();

    this->publishBaseB();
    res.success = true;
    res.message = std::to_string(this->alignTimer.ms()) + "ms";

    ROS_INFO("Alignment took time %fms", this->alignTimer.ms());
    return true;
}

void Rc4pcs::selectBaseB() {
    size_t i1, i2, i3, i4;
    Randomer random{0, this->setP.getPointCloud()->points.size() - 1};
    while (true) {
//        if (checkSameNum(i1 = random(), i2 = random(), i3 = random(), i4 = random())) {
        if (checkSameNum(i1 = 13152, i2 = 7309, i3 = 22969, i4 = 12225)) {
            // 11 candidates. Get points took time 332.932012ms. Alignment took time 334.734562ms
            // 11 candidates. Get points took time 158.729362ms. Alignment took time 162.169603ms. With the sphere cutting
            // 11 candidates. Get points took time 5.459865ms. Alignment took time 9.445370ms. With all intersections
            ROS_INFO("i1 %ld", i1);
            ROS_INFO("i2 %ld", i2);
            ROS_INFO("i3 %ld", i3);
            ROS_INFO("i4 %ld", i4);
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

void Rc4pcs::findCandidate(const Point &p1) {
    Timer debugTimer;
    debugTimer.start();

    std::vector<std::vector<Point*>> candidates(3);
    std::vector<std::vector<Condition*>> conditions(3);
    Condition conditionP1P2{nullptr, &this->baseB.getDescriptors()[0]};
    Condition conditionP1P3{nullptr, &this->baseB.getDescriptors()[1]};
    Condition conditionP1P4{nullptr, &this->baseB.getDescriptors()[2]};
    Condition conditionP2P3{nullptr, &this->baseB.getDescriptors()[3]};
    Condition conditionP2P4{nullptr, &this->baseB.getDescriptors()[4]};
    Condition conditionP3P4{nullptr, &this->baseB.getDescriptors()[5]};
    conditions[POINT::P2].push_back(&conditionP1P2);
    conditions[POINT::P3].push_back(&conditionP1P3);
    conditions[POINT::P3].push_back(&conditionP2P3);
    conditions[POINT::P4].push_back(&conditionP1P4);
    conditions[POINT::P4].push_back(&conditionP2P4);
    conditions[POINT::P4].push_back(&conditionP3P4);

    // TODO do it recursively
    conditionP1P2.point = &p1;
    conditionP1P3.point = &p1;
    conditionP1P4.point = &p1;
    this->octoMapQ.getPoints(conditions[POINT::P2], this->distanceThreshold, candidates[POINT::P2]);
    for (auto &p2 : candidates[POINT::P2]) {
        conditionP2P3.point = p2;
        conditionP2P4.point = p2;
        this->octoMapQ.getPoints(conditions[POINT::P3], this->distanceThreshold, candidates[POINT::P3]);
        for (auto &p3 : candidates[POINT::P3]) {
            conditionP3P4.point = p3;
            this->octoMapQ.getPoints(conditions[POINT::P4], this->distanceThreshold, candidates[POINT::P4]);
            for (auto &p4 : candidates[POINT::P4]) {
                ROS_INFO("I find a candidate");
            }
        }
    }

    debugTimer.stop();
    ROS_INFO("Get points took time %fms", debugTimer.ms());
}

// ROS
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
