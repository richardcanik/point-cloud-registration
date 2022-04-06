#ifndef SRC_RANDOMER_H
#define SRC_RANDOMER_H

#include <iostream>
#include <random>

class Randomer {
public:
    Randomer(const size_t &min, const size_t &max, const unsigned int seed = std::random_device{}()) :
        gen{seed},
        dist{min, max} {}

    size_t operator()() {
        return this->dist(this->gen);
    }

private:
    std::mt19937 gen;
    std::uniform_int_distribution<size_t> dist;
};

#endif //SRC_RANDOMER_H
