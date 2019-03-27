#include "SlalomDesigner.h"

#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>

std::ofstream of("out.csv");
std::ostream &os = of;

int main(void) {
  using namespace signal_processing;
#define SLALOM_NUM 5
#if SLALOM_NUM == 0
  auto constraint = SlalomDesigner::Constraint(M_PI / 2, 40, 45, 45);
#elif SLALOM_NUM == 1
  auto constraint = SlalomDesigner::Constraint(M_PI / 4, 30, 90, 45);
#elif SLALOM_NUM == 2
  auto constraint = SlalomDesigner::Constraint(M_PI / 2, 70, 90, 90);
#elif SLALOM_NUM == 3
  auto constraint = SlalomDesigner::Constraint(M_PI * 3 / 4, 80, 45, 90);
#elif SLALOM_NUM == 4
  auto constraint = SlalomDesigner::Constraint(M_PI, 90, 0, 90, 26);
#elif SLALOM_NUM == 5
  auto constraint = SlalomDesigner::Constraint(M_PI / 2, 40, 90 / std::sqrt(2),
                                               90 / std::sqrt(2));
#endif
  signal_processing::SlalomDesigner sd(constraint);
  const float v = 600;
  sd.reset(v);
  signal_processing::SlalomDesigner::State s;
  const float Ts = 0.001f;
  while (s.t < sd.t_end() + Ts) {
    os << s.t;
    os << "," << s.dddth;
    os << "," << s.ddth;
    os << "," << s.dth;
    os << "," << s.th;
    os << "," << s.dddx;
    os << "," << s.ddx;
    os << "," << s.dx;
    os << "," << s.x;
    os << "," << s.dddy;
    os << "," << s.ddy;
    os << "," << s.dy;
    os << "," << s.y;
    os << std::endl;
    sd.update(&s, Ts);
  }
  return 0;
}
