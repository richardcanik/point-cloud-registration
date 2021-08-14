#ifndef SRC_RC4PCS_H
#define SRC_RC4PCS_H

#include <chrono>
#include <localization/set.h>
#include <localization/base.h>
#include <localization/randomer.h>
#include <localization/matrix.h>

class Rc4pcs {
public:
    Rc4pcs(ros::NodeHandle& nodeHandle, const std::string& name);

private:
    bool align(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
    void selectBaseB();
    void computePointDistance(double seed = 0.5);
    void publishBaseB();
    void publishDebug(const Point &p);

    ros::ServiceServer alignService;
    ros::Publisher publisherBaseB;
    ros::Publisher publisherDebug;
    Set setP;
    Set setQ;
    Base baseB;
    Matrix matrix;
    double pointDistance;
};

#endif //SRC_RC4PCS_H
