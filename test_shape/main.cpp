#include "slalom.h"

#include <fstream>
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

void printDefinitions() {
  SS_SL90.printDefinition(std::cout, "SS_SL90");
  SS_SR90.printDefinition(std::cout, "SS_SR90");
  SS_FL45.printDefinition(std::cout, "SS_FL45");
  SS_FR45.printDefinition(std::cout, "SS_FR45");
  SS_FL90.printDefinition(std::cout, "SS_FL90");
  SS_FR90.printDefinition(std::cout, "SS_FR90");
  SS_FL135.printDefinition(std::cout, "SS_FL135");
  SS_FR135.printDefinition(std::cout, "SS_FR135");
  SS_FL180.printDefinition(std::cout, "SS_FL180");
  SS_FR180.printDefinition(std::cout, "SS_FR180");
  SS_FLV90.printDefinition(std::cout, "SS_FLV90");
  SS_FRV90.printDefinition(std::cout, "SS_FRV90");
  SS_FLS90.printDefinition(std::cout, "SS_FLS90");
  SS_FRS90.printDefinition(std::cout, "SS_FRS90");
}

void printCsv(const std::string &filebase, const slalom::Shape &ss,
              const float th_start = 0) {
  auto st = slalom::Trajectory(ss);
  const float v = 600;
  State s;
  st.reset(v, th_start, st.get_straight_prev() / v);
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
  float t = 0;
  const std::vector<float> ticks = {{
      st.getAccelDesigner().t_0(),
      st.getAccelDesigner().t_1(),
      st.getAccelDesigner().t_2(),
      st.getAccelDesigner().t_3(),
      st.t_end() + st.get_straight_post() / v,
  }};
  for (size_t i = 0; i < ticks.size(); ++i) {
    of = std::ofstream(filebase + "_" + std::to_string(i) + ".csv");
    while (t < ticks[i])
      st.update(s, t, Ts), printCSV(of, t, s), t += Ts;
  }
}

int main(void) {
  /* print definitions to stdout */
  printDefinitions();

  /* print trajectory to file*/
  printCsv("shape/shape_0", SS_SL90);
  printCsv("shape/shape_1", SS_FL45);
  printCsv("shape/shape_2", SS_FL90);
  printCsv("shape/shape_3", SS_FL135);
  printCsv("shape/shape_4", SS_FL180);
  printCsv("shape/shape_5", SS_FLV90, M_PI / 4);

  return 0;
}
