#ifndef SRC_RC4PCS_H
#define SRC_RC4PCS_H

#include <registration/set.h>
#include <registration/base.h>
#include <registration/randomer.h>
#include <registration/octo_map.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <thread_pool/thread_pool.hpp>

class Rc4pcs {
public:
    Rc4pcs(ros::NodeHandle &nodeHandle, const std::string &name);

private:
    bool align(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
    void selectBaseB();
    void findCandidate(const Point &p1);
    void computeOverlap(const Transform &t, const double ratio, double &overlap);
    void publishTransformedPointCloud(const Transform &transform);
    void publishBaseB();
    void publishTransformedBaseB(const Transform &transform);
    void publishBaseU(Base &baseU);

    ros::ServiceServer alignService;
    ros::Publisher publisherBaseB;
    ros::Publisher publisherBaseU;
    ros::Publisher publisherDebug1;
    ros::Publisher publisherDebug2;
    Set setP;
    Set setQ;
    Base baseB;
    OctoMap octoMapQ;
    double distanceThreshold;
    Timer alignTimer;
    bool stop;
    std::mutex stopMutex;
    thread_pool pool;
};

#endif //SRC_RC4PCS_H
