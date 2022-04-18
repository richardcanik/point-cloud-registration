#include <registration_core/cuda.cuh>
#include <iostream>
#include <vector>

__global__ void add(const int *a, const int *b, int *c) {
    c[threadIdx.x] = a[threadIdx.x] + b[threadIdx.x];
}

void test(int &c) {
    int *a, *b;  // host copies of a, b
    std::vector<int> va;
    int *d_a, *d_b, *d_c;   // device copies of a, b, c
    int size = N * sizeof(int);
    // Alloc space for device copies of a, b, c
    cudaMalloc((void **)&d_a, size);
    cudaMalloc((void **)&d_b, size);
    cudaMalloc((void **)&d_c, size);
    // Alloc space for host copies of a, b, c and setup input values
    a = (int *)malloc(size);
    b = (int *)malloc(size);
    a[0] = 1;
    b[0] = 1;
    // Copy inputs to device
    cudaMemcpy(d_a, a, size, cudaMemcpyHostToDevice);
    cudaMemcpy(d_b, b, size, cudaMemcpyHostToDevice);
    // Launch add() kernel on GPU with N threads
    add<<<1,N>>>(d_a, d_b, d_c);
    // Copy result back to host
    cudaMemcpy(&c, d_c, size, cudaMemcpyDeviceToHost);
    // Cleanup
    free(a);
    free(b);
    cudaFree(d_a);
    cudaFree(d_b);
    cudaFree(d_c);
}
