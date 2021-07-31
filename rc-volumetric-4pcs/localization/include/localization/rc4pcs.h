#ifndef SRC_RC4PCS_H
#define SRC_RC4PCS_H

#include <localization/set.h>
#include <localization/base.h>
#include <localization/randomer.h>

class Rc4pcs {
public:
    Rc4pcs(ros::NodeHandle& nodeHandle, const std::string& name);

private:
    bool align(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
    void selectBaseB();
    void publishBaseB();

    ros::ServiceServer alignService;
    ros::Publisher publisherBaseB;
    Set setP;
    Set setQ;
    Base baseB;
};

#endif //SRC_RC4PCS_H
