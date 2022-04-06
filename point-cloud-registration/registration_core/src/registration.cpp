#include <registration_core/registration.h>

Registration::Registration() :
        octoMapQ(3.0),
        distanceThreshold(0.3),
        aligned(false) {}

const Base &Registration::getBaseB() const {
    return this->baseB;
}

const std::vector<Result> &Registration::getResults() const {
    return this->results;
}

void Registration::align() {
    std::cout << "Alignment is starting" << std::endl;
    this->aligned = false;
    this->results.clear();
    this->alignTimer.start();
    this->selectBaseB();
    this->octoMapQ.fromSet(this->setQ);
    for (auto & point : this->setQ.getSet()) {
        this->pool.push_task([point, this] { this->findCandidate(point); });
    }
    this->pool.wait_for_tasks();
    this->alignTimer.stop();
    std::cout << "Alignment was stopped " << this->alignTimer.ms() << "ms" << std::endl;
}

void Registration::selectBaseB() {
    size_t i1, i2, i3, i4;
    Randomer random{0, this->setP.getSet().size() - 1};
    while (true) {
        if (areNumbersSame(i1 = random(), i2 = random(), i3 = random(), i4 = random())) {
//        if (areNumbersSame(i1 = 4231, i2 = 19731, i3 = 11209, i4 = 20663)) {
            std::cout << "i1: " << i1 << ", i2: " << i2 << ", i3: " << i3 << ", i4: " << i4 << std::endl;
            if (this->baseB.setBase(this->setP.getSet()[i1], this->setP.getSet()[i2],
                                    this->setP.getSet()[i3], this->setP.getSet()[i4],
                                    0.5*std::max(this->setP.getWidth(), this->setP.getHeight()))) {
                break;
            }
        }
    }
}

void Registration::findCandidate(const Point &p1) {
    double overlap;
    Base baseU;
    Matrix4 transform;
    std::vector<std::vector<const Point*>> candidates(3);
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
                transform = baseU.getFrame() * this->baseB.getFrame().inverse();
                this->computeOverlap(transform, 0.5, overlap);
                if (overlap > 20) {
                    this->alignTimer.stop();
                    this->alignmentMutex.lock();
                    if (!this->aligned) {
                        std::cout << "I find a candidate with the overlap: " << overlap << "%" << std::endl;
                        this->aligned = true;
                        this->results.push_back({overlap, this->alignTimer.ms(), transform, baseU});
                    }
                    this->alignmentMutex.unlock();
                }
                if (this->aligned) { break; }
            }
            if (this->aligned) { break; }
        }
        if (this->aligned) { break; }
    }
}

void Registration::computeOverlap(const Matrix4 &transform, const double &ratio, double &overlap) {
    const double delta = 2;
    std::vector<const Point*> points;
    unsigned long counter = 0;
    std::vector<Condition*> conditions;
    Condition condition{nullptr, &delta};
    Point p;

    conditions.push_back(&condition);
    const auto step = static_cast<size_t>(static_cast<double>(this->setP.getSet().size()) / (static_cast<double>(this->setP.getSet().size()) * ratio));
    for (size_t index = 0; index < this->setP.getSet().size(); index += step) {
        p = this->setP.getSet()[index];
        transformPoint(p, transform);
        condition.point = &p;
        this->octoMapQ.getPoints(conditions, 0.3, points);
        if (!points.empty()) {
            counter++;
        }
    }
    overlap = (static_cast<double>(counter) / (static_cast<double>(this->setP.getSet().size()) * ratio)) * 100.0;
}
