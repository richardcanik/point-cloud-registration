#ifndef SRC_TIMER_H
#define SRC_TIMER_H

#include <mutex>

class Timer {
public:
    void start(){
        this->start_time = std::chrono::steady_clock::now();
        this->stopMutex.lock();
        this->stopped = false;
        this->stopMutex.unlock();
    }

    void stop() {
        this->stopMutex.lock();
        if (!this->stopped) {
            this->elapsed_time = std::chrono::steady_clock::now() - this->start_time;
            this->stopped = true;
        }
        this->stopMutex.unlock();
    }

    [[nodiscard]] const double ms() const {
        return this->elapsed_time.count() * 1000;
    }
private:
    std::mutex stopMutex{};
    bool stopped = false;
    std::chrono::time_point<std::chrono::steady_clock> start_time = std::chrono::steady_clock::now();
    std::chrono::duration<double> elapsed_time = std::chrono::duration<double>::zero();
};

#endif //SRC_TIMER_H
