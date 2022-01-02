#include <localization/rc4pcs.h>

Rc4pcs::Rc4pcs(ros::NodeHandle &nodeHandle, const std::string &name) :
        alignService(nodeHandle.advertiseService(name + "/align", &Rc4pcs::align, this)),
        publisherBaseB(nodeHandle.advertise<visualization_msgs::Marker>(name + "/base_b", 1)),
        publisherBaseU(nodeHandle.advertise<visualization_msgs::Marker>(name + "/base_u", 1)),
        publisherDebug1(nodeHandle.advertise<sensor_msgs::PointCloud2>(name + "/debug1", 1)),
        publisherDebug2(nodeHandle.advertise<visualization_msgs::Marker>(name + "/debug2", 1)),
        setP(nodeHandle, name + "/source"),
        setQ(nodeHandle, name + "/destination"),
        octoMapQ(nodeHandle, 3.0),
        distanceThreshold(0.3),
        stop(false) {}

bool Rc4pcs::align(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
    ROS_INFO("Align source %s to destination %s", this->setP.getModelName().c_str(), this->setQ.getModelName().c_str());
    this->alignTimer.start();
    this->selectBaseB();
    this->octoMapQ.fromSet(this->setQ);
    this->stop = false;
    for (auto & point : this->setQ.getPointCloud()->points) {
        this->pool.push_task([point, this] {this->findCandidate(point);});
    }
    ROS_INFO("All %lu tasks pushed to the pool, running: %lu", this->pool.get_tasks_total(), this->pool.get_tasks_running());
    this->pool.wait_for_tasks();
    ROS_INFO("Total tasks in the pool: %lu , running: %lu", this->pool.get_tasks_total(), this->pool.get_tasks_running());

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
        if (checkSameNum(i1 = 4231, i2 = 19731, i3 = 11209, i4 = 20663)) {
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
    double overlap;
    Base baseU;
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
                baseU.setBase(p1, *p2, *p3, *p4);
                this->computeOverlap(baseU.getFrame() * this->baseB.getFrame().inverse(), 0.5, overlap);
                ROS_INFO("I find a candidate with the overlap: %f%%", overlap);
                if (overlap > 20) {
                    publishTransformedPointCloud(baseU.getFrame() * this->baseB.getFrame().inverse());
                    publishTransformedBaseB(baseU.getFrame() * this->baseB.getFrame().inverse());
                    publishBaseU(baseU);
                    this->stopMutex.lock();
                    this->stop = true;
                    this->alignTimer.stop();
                    this->stopMutex.unlock();
                }
                if (this->stop) { break; }
            }
            if (this->stop) { break; }
        }
        if (this->stop) { break; }
    }
}

void Rc4pcs::computeOverlap(const Transform &t, const double ratio, double &overlap) {
    const double delta = 2;
    std::vector<Point*> points;
    unsigned long counter = 0;
    std::vector<Condition*> conditions;
    Condition condition{nullptr, &delta};
    Point p;

    conditions.push_back(&condition);
    const size_t step = this->setP.getPointCloud()->points.size() / (this->setP.getPointCloud()->points.size() * ratio);
    for (size_t index = 0; index < this->setP.getPointCloud()->points.size(); index += step) {
        p = this->setP.getPointCloud()->points[index];
        transformPoint(p, t);
        condition.point = &p;
        this->octoMapQ.getPoints(conditions, 0.3, points);
        if (!points.empty()) {
            counter++;
        }
    }
    overlap = (static_cast<double>(counter) / (static_cast<double>(this->setP.getPointCloud()->points.size()) * ratio)) * 100.0;
}

// ROS
void Rc4pcs::publishTransformedPointCloud(const Transform &transform) {
    // Point Cloud
    sensor_msgs::PointCloud2 outputPointCloud;
    PointCloud::Ptr filteredPointCloud(new PointCloud), transformedPointCloud(new PointCloud);
    filterPointCloud(this->setP.getPointCloud(), filteredPointCloud);
    pcl::transformPointCloud(*filteredPointCloud, *transformedPointCloud, transform);
    pcl::toROSMsg(*transformedPointCloud, outputPointCloud);
    outputPointCloud.header.frame_id = "/base_link";
    this->publisherDebug1.publish(outputPointCloud);
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

void Rc4pcs::publishTransformedBaseB(const Transform &transform) {
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
    Point pb;
    for (int i : index) {
        pb = this->baseB.getPoints()[i];
        transformPoint(pb, transform);
        p.x = pb.x;
        p.y = pb.y;
        p.z = pb.z;
        lines.points.push_back(p);
    }
    this->publisherDebug2.publish(lines);
}

void Rc4pcs::publishBaseU(Base &baseU) {
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
    lines.color.g = 1.0;
    lines.color.a = 1.0;
    for (int i : index) {
        p.x = baseU.getPoints()[i].x;
        p.y = baseU.getPoints()[i].y;
        p.z = baseU.getPoints()[i].z;
        lines.points.push_back(p);
    }
    publisherBaseU.publish(lines);
}
