/**
 * @brief accel designer test
 * @author Ryotaro Onuki <kerikun11+github@gmail.com>
 * @date 2023-07-01
 * @copyright Copyright 2023 Ryotaro Onuki <kerikun11+github@gmail.com>
 */
#include <Arduino.h>
#include <ctrl/accel_designer.h>

#include <chrono>
#include <fstream>
#include <iostream>
#include <vector>

void app_ctrl_accel() {
  ctrl::AccelDesigner ad;
  const std::vector<std::vector<float>> params = {
      {100, 10, 4, 0, 2, 4},      //< vs -> vm -> vt, tm1>0, tm2>0
      {100, 10, 4, 0, 3, 4},      //< vs -> vm -> vt, tm1>0, tm2<0
      {100, 10, 4, 3, 0, 4},      //< vs -> vm -> vt, tm1<0, tm2>0
      {100, 10, 8, 0, 2, 4},      //< vs -> vr -> vt, vr<vm, tm1>0, tm2>0
      {100, 10, 8, 0, 6, 4},      //< vs -> vr -> vt, vr<vm, tm1>0, tm2<0
      {100, 10, 8, 0, 0.5, 0.2},  //< vs -> vr -> vt, vr<vm, tm1<0, tm2<0
      {100, 10, 6, 0, 3, 1},      //< vs -> vr -> vt, vr<vm, tm1>0, tm2<0
      {100, 10, 6, 0, 4, 1},      //< ve == vt, tm > 0 just
      {100, 10, 8, 0, 6, 1},      //< ve != vt, tm > 0, accel
      {100, 10, 8, 4, 0, 1},      //< ve != vt, tm > 0, decel
      {100, 10, 4, 0, 4, 0.1},    //< ve != vt, tm < 0, accel
      {100, 10, 4, 4, 0, 0.1},    //< ve != vt, tm < 0, decel
  };
  std::cout << std::endl;
  float sum = 0;
  for (const auto &ps : params) {
    const int n = 10000;
#if 0
    const auto ts = std::chrono::steady_clock::now();
    for (int i = 0; i < n; ++i) {
      ad.reset(ps[0], ps[1], ps[2], ps[3], ps[4], ps[5]);
      sum += ad.t_end();
    }
    const auto te = std::chrono::steady_clock::now();
    const auto dur =
        std::chrono::duration_cast<std::chrono::nanoseconds>(te - ts);
    std::cout << "Average Time: " << dur.count() / n << " [ns]" << std::endl;
#else  // arduino
    const auto ts = micros();
    for (int i = 0; i < n; ++i) {
      ad.reset(ps[0], ps[1], ps[2], ps[3], ps[4], ps[5]);
      sum += ad.t_end();
    }
    const auto te = micros();
    std::cout << "Average Time: " << (te - ts) / n << " [us]" << std::endl;
#endif
  }
  std::cout << sum << std::endl;
}
