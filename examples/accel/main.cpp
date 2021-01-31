/**
 * @file main.cpp
 * @author Ryotaro Onuki (kerikun11+github@gmail.com)
 * @brief This file evaluates the performance of acceleration curves.
 * @date 2020-05-04
 */
#define CTRL_LOG_LEVEL CTRL_LOG_LEVEL_INFO
#include <ctrl/accel_designer.h>

#include <chrono>
#include <fstream>
#include <iostream>
#include <vector>

void printCsv(const std::string &filebase, const ctrl::AccelDesigner &ad) {
  ctrl_logi << ad << std::endl;
  const float Ts = 1e-4f;
  std::ofstream of;
  const auto ticks = ad.getTimeStamp();
  float t = 0;
  for (size_t i = 0; i < ticks.size(); ++i) {
    of = std::ofstream(filebase + "_" + std::to_string(i) + ".csv");
    while (t + Ts < ticks[i]) {
      of << t;
      of << "," << ad.j(t);
      of << "," << ad.a(t);
      of << "," << ad.v(t);
      of << "," << ad.x(t);
      of << std::endl;
      t += Ts;
    }
  }
}

void measurement() {
  ctrl::AccelDesigner ad;
  const std::vector<std::vector<float>> params = {
      {100, 10, 4, 0, 2, 4},     //< vs -> vm -> vt, tm1>0, tm2>0
      {100, 10, 4, 0, 3, 4},     //< vs -> vm -> vt, tm1>0, tm2<0
      {100, 10, 4, 3, 0, 4},     //< vs -> vm -> vt, tm1<0, tm2>0
      {100, 10, 8, 0, 2, 4},     //< vs -> vr -> vt, vr<vm, tm1>0, tm2>0
      {100, 10, 8, 0, 6, 4},     //< vs -> vr -> vt, vr<vm, tm1>0, tm2<0
      {100, 10, 8, 0, 0.5, 0.2}, //< vs -> vr -> vt, vr<vm, tm1<0, tm2<0
      {100, 10, 6, 0, 3, 1},     //< vs -> vr -> vt, vr<vm, tm1>0, tm2<0
      {100, 10, 6, 0, 4, 1},     //< ve == vt, tm > 0 just
      {100, 10, 8, 0, 6, 1},     //< ve != vt, tm > 0, accel
      {100, 10, 8, 4, 0, 1},     //< ve != vt, tm > 0, decel
      {100, 10, 4, 0, 4, 0.1},   //< ve != vt, tm < 0, accel
      {100, 10, 4, 4, 0, 0.1},   //< ve != vt, tm < 0, decel
  };
  for (const auto &ps : params) {
    const int n = 10000;
    const auto ts = std::chrono::steady_clock::now();
    for (int i = 0; i < n; ++i)
      ad.reset(ps[0], ps[1], ps[2], ps[3], ps[4], ps[5]);
    const auto te = std::chrono::steady_clock::now();
    const auto dur =
        std::chrono::duration_cast<std::chrono::nanoseconds>(te - ts);
    std::cout << "Average Time: " << dur.count() / n << " [ns]" << std::endl;
  }
}

int main() {
  /* print csv */
  ctrl::AccelDesigner ad;
  ad.reset(100, 10, 4, 0, 2, 4);
  printCsv("accel", ad);

  /* time measurement */
  measurement();

  return 0;
}
