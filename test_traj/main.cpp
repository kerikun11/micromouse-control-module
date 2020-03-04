#include "straight.h"
#include "trajectory_tracker.h"

#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>

std::ofstream of("out.csv");
std::ostream &os = of;

using namespace ctrl;

int main(void) {
  const float Ts = 0.001f;
  const float jerk = 240000;
  const float accel = 6000;
  const float v_start = 0;
  const float v_max = 1200;
  const float v_end = 0;
  const float distance = 90 * 8;
  TrajectoryTracker::Gain gain;
  TrajectoryTracker tt(gain);
  straight::Trajectory ad;
  State s;
  ad.reset(jerk, accel, v_start, v_max, v_end, distance);
  tt.reset(v_start);
  for (float t = 0; t < ad.t_end(); t += Ts) {
    ad.update(s, t);
    auto est_q = s.q;
    auto est_v = s.dq.x;
    auto est_a = s.ddq.x;
    auto ref = tt.update(est_q, est_v, est_a, s);
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
