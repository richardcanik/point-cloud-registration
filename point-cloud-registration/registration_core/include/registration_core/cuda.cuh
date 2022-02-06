#include <cuda_runtime.h>

#define N 1

__global__ void add(const int *a, const int *b, int *c);

void test(int &c);
