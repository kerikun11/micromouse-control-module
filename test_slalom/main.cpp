#include <cmath>
#include <fstream>
#include <iostream>
#include <string>

#include "slalom.h"
#include "trajectory_tracker.h"

std::ofstream of("out.csv");
std::ostream &os = of;

using namespace ctrl;

static auto SS_SL90 = slalom::Shape(Position(45, 45, M_PI / 2), 44);
static auto SS_SR90 = slalom::Shape(Position(45, -45, -M_PI / 2), -44);
static auto SS_FL45 = slalom::Shape(Position(90, 45, M_PI / 4), 31);
static auto SS_FR45 = slalom::Shape(Position(90, -45, -M_PI / 4), -31);
static auto SS_FL90 = slalom::Shape(Position(90, 90, M_PI / 2), 70);
static auto SS_FR90 = slalom::Shape(Position(90, -90, -M_PI / 2), -70);
static auto SS_FL135 = slalom::Shape(Position(45, 90, M_PI * 3 / 4), 80);
static auto SS_FR135 = slalom::Shape(Position(45, -90, -M_PI * 3 / 4), -80);
static auto SS_FL180 = slalom::Shape(Position(0, 90, M_PI), 90, 24);
static auto SS_FR180 = slalom::Shape(Position(0, -90, -M_PI), -90, 24);
static auto SS_FLV90 =
    slalom::Shape(Position(45 * std::sqrt(2), 45 * std::sqrt(2), M_PI / 2), 48);
static auto SS_FRV90 = slalom::Shape(
    Position(45 * std::sqrt(2), -45 * std::sqrt(2), -M_PI / 2), -48);
static auto SS_FLS90 = slalom::Shape(Position(45, 45, M_PI / 2), 44);
static auto SS_FRS90 = slalom::Shape(Position(45, -45, -M_PI / 2), -44);

#define TO_STRING(VariableName) #VariableName

int main(void) {
#define SLALOM_NUM 5
#if SLALOM_NUM == 0
  auto sd = slalom::Trajectory(SS_SL90);
#elif SLALOM_NUM == 1
  auto sd = slalom::Trajectory(SS_FL45);
#elif SLALOM_NUM == 2
  auto sd = slalom::Trajectory(SS_FL90);
#elif SLALOM_NUM == 3
  auto sd = slalom::Trajectory(SS_FL135);
#elif SLALOM_NUM == 4
  auto sd = slalom::Trajectory(SS_FL180);
#elif SLALOM_NUM == 5
  auto sd = slalom::Trajectory(SS_FLV90);
#endif
  std::cout << sd.getShape() << std::endl;

  TrajectoryTracker::Gain gain;
  TrajectoryTracker tt(gain);
  const float v = 600;
  sd.reset(v);
  tt.reset(v);
  State s;
  const float Ts = 0.001f;
  for (float t = 0; t < sd.t_end(); t += Ts) {
    sd.update(s, t, Ts);
    auto est_q = Position(0, 0, 0);
    auto est_v = Polar(0, 0);
    auto est_a = Polar(0, 0);
    auto ref = tt.update(est_q, est_v, est_a, s.q, s.dq, s.ddq, s.dddq);
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
