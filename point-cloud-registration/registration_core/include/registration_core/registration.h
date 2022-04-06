#ifndef SRC_REGISTRATION_H
#define SRC_REGISTRATION_H

#include <registration_core/base.h>
#include <registration_core/set.h>
#include <registration_core/octo_map.h>
#include <registration_core/randomer.h>
#include <registration_core/timer.h>
#include <thread_pool/thread_pool.hpp>

struct Result {
    double overlap;
    double alignmentTime;
    Matrix4 transform;
    Base baseU;
};

class Registration {
public:
    Registration();

protected:
    void align();
    [[nodiscard]] const Base &getBaseB() const;
    [[nodiscard]] const std::vector<Result> &getResults() const;

    Set setP;
    Set setQ;

private:
    void selectBaseB();
    void findCandidate(const Point &p1);
    void computeOverlap(const Matrix4 &transform, const double &ratio, double &overlap);

    Base baseB;
    OctoMap octoMapQ;
    double distanceThreshold;
    Timer alignTimer;
    bool aligned;
    std::mutex alignmentMutex;
    thread_pool pool;
    std::vector<Result> results;
};

#endif //SRC_REGISTRATION_H
