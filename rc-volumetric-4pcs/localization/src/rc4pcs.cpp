#include <localization/rc4pcs.h>

Rc4pcs::Rc4pcs(ros::NodeHandle &nodeHandle, const std::string& name) :
    alignService(nodeHandle.advertiseService(name + "/align", &Rc4pcs::align, this)),
    publisherBaseB(nodeHandle.advertise<visualization_msgs::Marker>(name + "/base_b", 1)),
    setP(nodeHandle, name + "/source"),
    setQ(nodeHandle, name + "/destination") {}

bool Rc4pcs::align(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
    this->selectBaseB();
    this->publishBaseB();
    res.success = true;
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
    visualization_msgs::Marker line;
    geometry_msgs::Point p;
    std::vector<int> index{0, 1, 2, 3, 0, 2, 1, 3};

    line.header.frame_id = "base_link";
    line.header.stamp = ros::Time::now();
    line.ns = "base";
    line.action = visualization_msgs::Marker::ADD;
    line.pose.orientation.w = 1.0;
    line.id = 0;
    line.type = visualization_msgs::Marker::LINE_STRIP;
    line.color.b = 1.0;
    line.color.a = 1.0;
    for (int i : index) {
        p.x = this->baseB.getPoints()[i].x;
        p.y = this->baseB.getPoints()[i].y;
        p.z = this->baseB.getPoints()[i].z;
        line.points.push_back(p);
    }
    publisherBaseB.publish(line);
}