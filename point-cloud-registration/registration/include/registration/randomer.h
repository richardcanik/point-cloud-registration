#ifndef SRC_RANDOMER_H
#define SRC_RANDOMER_H

#include <iostream>
#include <random>

class Randomer {
public:
    Randomer(size_t min, size_t max, unsigned int seed = std::random_device{}())
            : gen{seed}, dist{min, max} {
    }

    void setSeed(unsigned int seed) {
        this->gen.seed(seed);
    }

    size_t operator()() {
        return this->dist(this->gen);
    }

private:
    std::mt19937 gen;
    std::uniform_int_distribution<size_t> dist;
};

#endif //SRC_RANDOMER_H
