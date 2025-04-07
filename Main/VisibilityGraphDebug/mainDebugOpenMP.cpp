#include <omp.h>
#include <Utils/Utils.h>

int main() {

  /* Fork a team of threads giving them their own copies of variables */
  #pragma omp parallel default(none)
  {
    /* Obtain thread number */
    int tid = omp_get_thread_num();
    printf("Hello World from thread = %d\n", tid);

    /* Only master thread does this */
    if (tid == 0) {
      int nthreads = omp_get_num_threads();
      printf("Number of threads = %d\n", nthreads);
    }
  }  /* All threads join master thread and disband */

  return 0;
}