#include "straight.h"
#include "trajectory_tracker.h"

#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>

std::ofstream of("trajectory.csv");
std::ostream &os = of;

using namespace ctrl;

void printCsv(const float t, const State &s,
              const TrajectoryTracker::Result &ref) {
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

int main(void) {
  /* constants */
  const float Ts = 0.001f;
  const float jerk = 240000;
  const float accel = 6000;
  const float v_max = 1200;
  const float v_slalom = 600;
  /* trajectory tracker */
  TrajectoryTracker::Gain gain;
  TrajectoryTracker tt(gain);
  /* state variables */
  State s;
  /* init */
  const float v_start = 0;
  const float d_straight = 90 * 4;
  {
    straight::Trajectory trajectory;
    trajectory.reset(jerk, accel, v_start, v_max, v_slalom, d_straight);
    tt.reset(v_start);
    for (float t = 0; t < trajectory.t_end(); t += Ts) {
      trajectory.update(s, t);
      const auto est_q = s.q;
      const auto est_v = s.dq.x;
      const auto est_a = s.ddq.x;
      const auto ref = tt.update(est_q, est_v, est_a, s);
      printCsv(t, s, ref);
    }
  }
  return 0;
}
