#ifndef SRC_TIMER_H
#define SRC_TIMER_H

class Timer {
public:
    void start(){
        this->start_time = std::chrono::steady_clock::now();
    }

    void stop() {
        this->elapsed_time = std::chrono::steady_clock::now() - this->start_time;
    }

    double ms() const {
        return this->elapsed_time.count() * 1000;
    }
private:
    std::chrono::time_point<std::chrono::steady_clock> start_time = std::chrono::steady_clock::now();
    std::chrono::duration<double> elapsed_time = std::chrono::duration<double>::zero();
};

#endif //SRC_TIMER_H
