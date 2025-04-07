#include <iostream>
#include <chrono>
#include <immintrin.h>

// Array size for benchmarking
constexpr size_t SIZE = 1<<28; // 100 million elements


// Regular addition function
void regular_addition(const float* a, const float* b, const float *c, float* result, size_t size) {
    for (size_t i = 0; i < size; ++i) {
        result[i] = a[i] * b[i] + c[i];
    }
}

// AVX2 addition function
void avx2_addition(const float* A, const float* B, const float* C, float* result, size_t size) {
    __m256 * a = (__m256*)A;
    __m256 * b = (__m256*)B;
    __m256 * c = (__m256*)C;
    __m256 * r = (__m256*)result;
    const size_t n = size / 8;
    for (size_t i = 0; i < n; i++)
    {
        r[i] = _mm256_fmadd_ps(a[i], b[i], c[i]); 
    }

    // Handle remaining elements (if any)
    // for (; i < size; ++i) {
    //     result[i] = a[i] + b[i];
    // }
}

int main() {
    // Initialize arrays
    float * a = (float*)aligned_alloc(64, SIZE*sizeof(float));
    float * b = (float*)aligned_alloc(64, SIZE*sizeof(float));
    float * c = (float*)aligned_alloc(64, SIZE*sizeof(float));
    float * result_regular = (float*) aligned_alloc(64, SIZE*sizeof(float));
    float * result_avx2 = (float*) aligned_alloc(64, SIZE*sizeof(float));
    // float* result_avx2 = (float*)aligned_alloc(256, SIZE);

    // Initialize input arrays with random values
    for (size_t i = 0; i < SIZE; ++i) {
        a[i] = static_cast<float>(rand()) / RAND_MAX;
        b[i] = static_cast<float>(rand()) / RAND_MAX;
        c[i] = static_cast<float>(rand()) / RAND_MAX;
    }

    // Benchmark regular addition
    auto start = std::chrono::high_resolution_clock::now();
    regular_addition(a, b, c, result_regular, SIZE);
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> regular_time = end - start;
    std::cout << "Regular addition time: " << regular_time.count() << " seconds" << std::endl;

    // Benchmark AVX2 addition
    start = std::chrono::high_resolution_clock::now();
    avx2_addition(a, b, c, result_avx2, SIZE);
    // avx2_addition(A, B, result_avx2, SIZE);
    end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> avx2_time = end - start;
    std::cout << "AVX2 addition time: " << avx2_time.count() << " seconds" << std::endl;

    // Verify results
    for (size_t i = 0; i < SIZE; ++i) {
        if (result_regular[i] != result_avx2[i]) {
            std::cerr << "Mismatch at index " << i << std::endl;
            return -1;
        }
    }

    std::cout << "Results verified: AVX2 produces the same result as the regular addition." << std::endl;

    // Clean up
    free(a);
    free(b);
    free(result_avx2);
    free(result_regular);

    return 0;
}
