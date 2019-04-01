#include "AccelDesigner.h"

#include <chrono>
#include <fstream>
#include <iostream>
#include <random>

ctrl::AccelDesigner ad;
std::ofstream of("out.csv");

void test(float jm, float am, float vs, float va, float ve, float d, float xs,
          float ts) {
  ad.reset(jm, am, vs, va, ve, d, xs, ts);
  ad.printCsv(of);
  // std::cout << ad << std::endl;
}

int main(void) {
  // test(500000, 9000, 0, 1200, 0, -360, 0, 0);
#if 1
  int n = 100;
  std::mt19937 mt{std::random_device{}()};
  std::uniform_real_distribution<float> j_urd(100000, 500000);
  std::uniform_real_distribution<float> a_urd(1000, 18000);
  std::uniform_real_distribution<float> v_urd(90, 4800);
  std::uniform_real_distribution<float> d_urd(0, 32 * 90);
  auto ts = std::chrono::steady_clock::now();
  for (int i = 0; i < n; ++i) {
    test(j_urd(mt), a_urd(mt), ad.v_end(), v_urd(mt), v_urd(mt), d_urd(mt),
         ad.x_end(), ad.t_end());
  }
  auto te = std::chrono::steady_clock::now();
  auto dur = std::chrono::duration_cast<std::chrono::nanoseconds>(te - ts);
  std::cout << "Average Time: " << dur.count() / n << " [ns]" << std::endl;
#endif

  return 0;
}
