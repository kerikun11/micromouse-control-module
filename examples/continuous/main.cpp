/**
 * @file main.cpp
 * @author Ryotaro Onuki (kerikun11+github@gmail.com)
 * @brief This file evaluates the performance of acceleration curves.
 * @date 2020-05-04
 */
#include <ctrl/accel_designer.h>

#include <chrono>
#include <fstream>
#include <iostream>
#include <random>

ctrl::AccelDesigner ad;
ctrl::AccelCurve ac;
std::ofstream of("continuous.csv");

void test(const float jm, const float am, const float vm, const float vs,
          const float vt, const float d, const float xs, const float ts) {
  ad.reset(jm, am, vm, vs, vt, d, xs, ts);
  ad.printCsv(of);
  // std::cout << ad << std::endl;
}

int main(void) {
  // test(4800 * M_PI, 48 * M_PI, 4 * M_PI, 0, 0, M_PI / 2, 0, 0);
  // test(240000, 3600, 720, ad.v_end(), 0, 90, ad.x_end(), ad.t_end());
#if 1
  int n = 100;
  std::mt19937 mt{std::random_device{}()};
  std::uniform_real_distribution<float> j_urd(10000, 1000000);
  std::uniform_real_distribution<float> a_urd(1000, 100000);
  std::uniform_real_distribution<float> v_urd(100, 10000);
  std::uniform_real_distribution<float> d_urd(0, 32 * 90);
  auto ts = std::chrono::steady_clock::now();
  for (int i = 0; i < n; ++i) {
    const auto sign = 1;
    test(j_urd(mt), a_urd(mt), v_urd(mt), ad.v_end(), sign * v_urd(mt),
         sign * d_urd(mt), ad.x_end(), ad.t_end());
  }
  auto te = std::chrono::steady_clock::now();
  auto dur = std::chrono::duration_cast<std::chrono::nanoseconds>(te - ts);
  std::cout << "Average Time: " << dur.count() / n << " [ns]" << std::endl;
#endif

  return 0;
}
