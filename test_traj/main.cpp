#include "SlalomDesigner.h"
#include "TrajectoryTracker.h"

#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>

std::ofstream of("out.csv");
std::ostream &os = of;

using namespace ctrl;

SlalomDesigner sd_SL90(SlalomDesigner::Shape(Position(45, 45, M_PI / 2), 40));
SlalomDesigner sd_FL45(SlalomDesigner::Shape(Position(90, 45, M_PI / 4), 30));
SlalomDesigner sd_FL90(SlalomDesigner::Shape(Position(90, 90, M_PI / 2), 70));
SlalomDesigner sd_FL135(SlalomDesigner::Shape(Position(45, 90, M_PI * 3 / 4),
                                              80));
SlalomDesigner sd_FL180(SlalomDesigner::Shape(Position(0, 90, M_PI), 90, 30));
SlalomDesigner sd_FLV90(SlalomDesigner::Shape(
    Position(45 * std::sqrt(2), 45 * std::sqrt(2), M_PI / 2), 42));

int main(void) {
  std::cout << sd_SL90 << std::endl;
  std::cout << sd_FL45 << std::endl;
  std::cout << sd_FL90 << std::endl;
  std::cout << sd_FL135 << std::endl;
  std::cout << sd_FL180 << std::endl;
  std::cout << sd_FLV90 << std::endl;

#define SLALOM_NUM 5
#if SLALOM_NUM == 0
  auto &sd = sd_SL90;
#elif SLALOM_NUM == 1
  auto &sd = sd_FL45;
#elif SLALOM_NUM == 2
  auto &sd = sd_FL90;
#elif SLALOM_NUM == 3
  auto &sd = sd_FL135;
#elif SLALOM_NUM == 4
  auto &sd = sd_FL180;
#elif SLALOM_NUM == 5
  auto &sd = sd_FLV90;
#endif
  TrajectoryTracker tt;
  const float v = 600;
  sd.reset(v);
  tt.reset(v);
  SlalomDesigner::State s;
  const float Ts = 0.001f;
  while (s.t < sd.t_end()) {
    sd.update(&s, Ts);
    auto est_q = Position(0, 0, 0);
    auto est_v = Polar(0, 0);
    auto est_a = Polar(0, 0);
    auto ref = tt.update(est_q, est_v, est_a, s.q, s.dq, s.ddq, s.dddq);
    os << s.t;
    os << "," << s.dddq.th;
    os << "," << s.ddq.th;
    os << "," << s.dq.th;
    os << "," << s.q.th;
    os << "," << s.dddq.x;
    os << "," << s.ddq.x;
    os << "," << s.dq.x;
    os << "," << s.q.x;
    os << "," << s.dddq.y;
    os << "," << s.ddq.y;
    os << "," << s.dq.y;
    os << "," << s.q.y;
    os << "," << ref.v;
    os << "," << ref.w;
    os << "," << ref.dv;
    os << "," << ref.dw;
    os << std::endl;
  }
  return 0;
}
