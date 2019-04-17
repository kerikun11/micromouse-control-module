#include "TrajectoryTracker.h"
#include "slalom.h"

#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>

std::ofstream of("out.csv");
std::ostream &os = of;

using namespace ctrl;

int main(void) {
  const float jerk = 500000;
  const float accel = 3000;
  const float v_start = 0;
  const float v_max = 1200;
  const float v_end = 0;
  const float distance = 90 * 8;
  AccelDesigner ad(jerk, accel, v_start, v_max, v_end, distance);
  TrajectoryTracker tt;
  tt.reset(v_start);
  slalom::State s;
  for (float t = 0; t < ad.t_end(); t += 0.001f) {
    auto est_q = Position(0, 0, 0);
    auto est_v = Polar(0, 0);
    auto est_a = Polar(0, 0);
    auto ref_q = Position(ad.x(t), 0);
    auto ref_dq = Position(ad.v(t), 0);
    auto ref_ddq = Position(ad.a(t), 0);
    auto ref_dddq = Position(ad.j(t), 0);
    auto ref = tt.update(est_q, est_v, est_a, ref_q, ref_dq, ref_ddq, ref_dddq);
    os << t;
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
