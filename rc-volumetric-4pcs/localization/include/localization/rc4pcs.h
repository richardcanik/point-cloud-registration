#ifndef SRC_RC4PCS_H
#define SRC_RC4PCS_H

#include <localization/set.h>
#include <localization/base.h>
#include <localization/randomer.h>
#include <localization/octo_map.h>

class Rc4pcs {
public:
    Rc4pcs(ros::NodeHandle& nodeHandle, const std::string& name);

private:
    bool align(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
    void selectBaseB();
    void publishBaseB();
    void publishDebug(const std::vector<Point*> &p);

    ros::ServiceServer alignService;
    ros::Publisher publisherBaseB;
    ros::Publisher publisherDebug;
    Set setP;
    Set setQ;
    Base baseB;
    OctoMap octoMapQ;
    double distanceThreshold;
    Timer alignTimer;
};

#endif //SRC_RC4PCS_H
