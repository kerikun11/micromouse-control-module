#include "SlalomDesigner.h"
#include "TrajectoryTracker.h"

#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>

std::ofstream of("out.csv");
std::ostream &os = of;

using namespace ctrl;
// using namespace slalom;

// static auto SS_SL90 = slalom::Shape(Position(45, 45, M_PI / 2), 40);
// static auto SS_FL45 = slalom::Shape(Position(90, 45, M_PI / 4), 25);
// static auto SS_FL90 = slalom::Shape(Position(90, 90, M_PI / 2), 60);
// static auto SS_FL135 = slalom::Shape(Position(45, 90, M_PI * 3 / 4), 80);
// static auto SS_FL180 = slalom::Shape(Position(0, 90, M_PI), 90, 30);
// static auto SS_FLV90 =
//     slalom::Shape(Position(45 * std::sqrt(2), 45 * std::sqrt(2), M_PI / 2),
//     42);

static const auto SS_SL90 = ctrl::slalom::Shape(
    ctrl::Position(45, 45, 1.5708), ctrl::Position(39.9966, 39.9986, 1.5708),
    5.00339, 5.00138, 372.034);
static const auto SS_FL45 = ctrl::slalom::Shape(
    ctrl::Position(90, 45, 0.785398),
    ctrl::Position(60.3412, 25.0001, 0.785398), 9.65883, 28.2842, 586.486);
static const auto SS_FL90 = ctrl::slalom::Shape(
    ctrl::Position(90, 90, 1.5708), ctrl::Position(59.995, 60.0004, 1.5708),
    30.005, 29.9996, 558.028);
static const auto SS_FL135 = ctrl::slalom::Shape(
    ctrl::Position(45, 90, 2.35619), ctrl::Position(33.1317, 79.9992, 2.35619),
    21.8691, 14.1433, 510.551);
static const auto SS_FL180 =
    ctrl::slalom::Shape(ctrl::Position(0, 90, 3.14159),
                        ctrl::Position(0, 90, 3.14159), 30, 30, 557.896);
static const auto SS_FLV90 = ctrl::slalom::Shape(
    ctrl::Position(63.6396, 63.6396, 1.5708),
    ctrl::Position(41.9965, 41.9999, 1.5708), 21.6431, 21.6397, 390.623);

#define TO_STRING(VariableName) #VariableName

int main(void) {
  SS_SL90.printDefinition(std::cout, TO_STRING(SS_SL90));
  SS_FL45.printDefinition(std::cout, TO_STRING(SS_FL45));
  SS_FL90.printDefinition(std::cout, TO_STRING(SS_FL90));
  SS_FL135.printDefinition(std::cout, TO_STRING(SS_FL135));
  SS_FL180.printDefinition(std::cout, TO_STRING(SS_FL180));
  SS_FLV90.printDefinition(std::cout, TO_STRING(SS_FLV90));

#define SLALOM_NUM 1
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
  TrajectoryTracker tt;
  const float v = 600;
  sd.reset(v);
  tt.reset(v);
  slalom::State s;
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
