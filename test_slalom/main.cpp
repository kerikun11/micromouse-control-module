#include "slalom.h"

#include <cmath>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

using namespace ctrl;

static auto SS_SL90 = slalom::Shape(Position(45, 45, M_PI / 2), 44);
static auto SS_SR90 = slalom::Shape(Position(45, -45, -M_PI / 2), -44);
static auto SS_FL45 = slalom::Shape(Position(90, 45, M_PI / 4), 30);
static auto SS_FR45 = slalom::Shape(Position(90, -45, -M_PI / 4), -30);
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

int main(void) {
#define SLALOM_NUM 5
#if SLALOM_NUM == 0
  auto st = slalom::Trajectory(SS_SL90);
#elif SLALOM_NUM == 1
  auto st = slalom::Trajectory(SS_FL45);
#elif SLALOM_NUM == 2
  auto st = slalom::Trajectory(SS_FL90);
#elif SLALOM_NUM == 3
  auto st = slalom::Trajectory(SS_FL135);
#elif SLALOM_NUM == 4
  auto st = slalom::Trajectory(SS_FL180);
#elif SLALOM_NUM == 5
  auto st = slalom::Trajectory(SS_FLV90);
#endif
  std::cout << st.getShape() << std::endl;

  const float v = 600;
  State s;
  st.reset(v, M_PI / 4);
  const float Ts = 0.00001f;
  const auto printCSV = [](std::ostream &os, const float t, const State &s) {
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
    os << std::endl;
  };
  std::ofstream of;
  float t = -st.get_straight_prev() / v;
  int i = 0;
  of = std::ofstream("slalom_" + std::to_string(i++) + ".csv");
  while (t < 0)
    st.update(s, t, Ts), printCSV(of, t, s), t += Ts;
  of = std::ofstream("slalom_" + std::to_string(i++) + ".csv");
  while (t < st.getAccelDesigner().t_1())
    st.update(s, t, Ts), printCSV(of, t, s), t += Ts;
  of = std::ofstream("slalom_" + std::to_string(i++) + ".csv");
  while (t < st.getAccelDesigner().t_2())
    st.update(s, t, Ts), printCSV(of, t, s), t += Ts;
  of = std::ofstream("slalom_" + std::to_string(i++) + ".csv");
  while (t < st.getAccelDesigner().t_3())
    st.update(s, t, Ts), printCSV(of, t, s), t += Ts;
  of = std::ofstream("slalom_" + std::to_string(i++) + ".csv");
  while (t < st.t_end() + st.get_straight_post() / v)
    st.update(s, t, Ts), printCSV(of, t, s), t += Ts;

  return 0;
}
