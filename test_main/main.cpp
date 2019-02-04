#include "AccelDesigner.h"
#include <chrono>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <random>

AccelDesigner sd;
std::ofstream of("out.csv");

void test(float am, float vs, float va, float ve, float d, float xs, float ts) {
  sd.reset(am, vs, va, ve, d, xs, ts);
  sd.printCsv(of);
  // std::cout << "main x_end: " << sd.x_end() << std::endl;
}

int main(void) {
  // test(9000, -1, 2400, 0, 90 * 4, 1, sd.t_end());
  // test(9000, sd.v_end(), 1, 0, 1, sd.x_end(), sd.t_end());
  // test(9000, sd.v_end(), 1200, 900, 90, sd.x_end(), sd.t_end());
  // test(9000, sd.v_end(), 1200, 300, 360, sd.x_end(), sd.t_end());
  // test(9000, sd.v_end(), 1200, 1200, 90, sd.x_end(), sd.t_end());
  // test(9000, sd.v_end(), 1200, 300, 360, sd.x_end(), sd.t_end());
  // test(9000, sd.v_end(), 3600, 1200, 2880, sd.x_end(), sd.t_end());
  // test(9000, sd.v_end(), 1200, 0, 90, sd.x_end(), sd.t_end());
  // test(9000, sd.v_end(), 1200, 0, 90, sd.x_end(), sd.t_end());
  // test(9000, sd.v_end(), 1200, 0, 90, sd.x_end(), sd.t_end());
  // test(9000, sd.v_end(), 1200, 0, 90, sd.x_end(), sd.t_end());
  // test(9000, sd.v_end(), 1200, 0, 90, sd.x_end(), sd.t_end());
  // test(9000, sd.v_end(), 3600, 1200, 2880, sd.x_end(), sd.t_end());
  // test(9000, sd.v_end(), 1200, 0, 90, sd.x_end(), sd.t_end());
  // test(6000, sd.v_end(), 2400, 1200, 1800, sd.x_end(), sd.t_end());
  // test(6000, sd.v_end(), 2400, 600, 360, sd.x_end(), sd.t_end());
  // test(6000, sd.v_end(), 2400, 0, 720, sd.x_end(), sd.t_end());

  // test(6000, 240, 1200, 480, 720, sd.x_end(), sd.t_end());
  // test(6000, sd.v_end(), 1200, 0, -90, sd.x_end(), sd.t_end());
  // test(6000, sd.v_end(), 1200, 0, 720, sd.x_end(), sd.t_end());
  // test(6000, 2400, 1200, 0, 90, sd.x_end(), sd.t_end());
  // test(4318.56, 1923.44, 269.985, 979.856, 67.8627, sd.x_end(), sd.t_end());

  // sd.reset(9000, 120, 120, 120, 0, 0, 0);
  int n = 100;
  std::mt19937 mt{std::random_device{}()};
  std::uniform_real_distribution<float> a_urd(1000, 9000);
  std::uniform_real_distribution<float> v_urd(90, 3600);
  std::uniform_real_distribution<float> d_urd(9, 900);
  auto ts = std::chrono::steady_clock::now();
  for (int i = 0; i < n; ++i) {
    test(a_urd(mt), sd.v_end(), v_urd(mt), v_urd(mt), d_urd(mt), sd.x_end(),
         sd.t_end());
  }
  auto te = std::chrono::steady_clock::now();
  auto dur = std::chrono::duration_cast<std::chrono::nanoseconds>(te - ts);
  std::cout << "Average Time: " << dur.count() / n << " [ns]" << std::endl;

  return 0;
}
