#include "slalom.h"

#include <fstream>
#include <string>
#include <vector>

using namespace ctrl;

static auto SS_S90L = slalom::Shape(Position(45, 45, M_PI / 2), 44);
static auto SS_S90R = slalom::Shape(Position(45, -45, -M_PI / 2), -44);
static auto SS_F45L = slalom::Shape(Position(90, 45, M_PI / 4), 30);
static auto SS_F45R = slalom::Shape(Position(90, -45, -M_PI / 4), -30);
static auto SS_F90L = slalom::Shape(Position(90, 90, M_PI / 2), 70);
static auto SS_F90R = slalom::Shape(Position(90, -90, -M_PI / 2), -70);
static auto SS_F135L = slalom::Shape(Position(45, 90, M_PI * 3 / 4), 80);
static auto SS_F135R = slalom::Shape(Position(45, -90, -M_PI * 3 / 4), -80);
static auto SS_F180L = slalom::Shape(Position(0, 90, M_PI), 90, 24);
static auto SS_F180R = slalom::Shape(Position(0, -90, -M_PI), -90, 24);
static auto SS_FV90L =
    slalom::Shape(Position(45 * std::sqrt(2), 45 * std::sqrt(2), M_PI / 2), 48);
static auto SS_FV90R = slalom::Shape(
    Position(45 * std::sqrt(2), -45 * std::sqrt(2), -M_PI / 2), -48);
static auto SS_FS90L = slalom::Shape(Position(45, 45, M_PI / 2), 44);
static auto SS_FS90R = slalom::Shape(Position(45, -45, -M_PI / 2), -44);

static auto SS_FK90L = slalom::Shape(
    Position(90 * std::sqrt(2), 90 * std::sqrt(2), M_PI / 2), 125);
static auto SS_FK90R = slalom::Shape(
    Position(90 * std::sqrt(2), -90 * std::sqrt(2), -M_PI / 2), -125);

void printDefinition(std::ostream &os, const std::string &name,
                     const slalom::Shape &s) {
  const AccelDesigner ad(s.m_dddth, s.m_ddth, 0, s.m_dth, 0, s.total.th);
  const auto t_total =
      ad.t_end() + (s.straight_prev + s.straight_post) / s.v_ref;
  int width = 9;
  os << "/* " << std::setfill(' ') << std::setw(8) << name
     << " Total Time:" << std::setfill(' ') << std::setw(width) << t_total
     << " [s] */" << std::endl;
  os << "static const auto " << std::setfill(' ') << std::setw(8) << name;
  os << " = ctrl::slalom::Shape(";
  os << "ctrl::Position(" << std::setfill(' ') << std::setw(width) << s.total.x
     << ", " << std::setfill(' ') << std::setw(width) << s.total.y << ", "
     << std::setfill(' ') << std::setw(width) << s.total.th << "), ";
  os << "ctrl::Position(" << std::setfill(' ') << std::setw(width) << s.curve.x
     << ", " << std::setfill(' ') << std::setw(width) << s.curve.y << ", "
     << std::setfill(' ') << std::setw(width) << s.curve.th << "), ";
  os << std::setfill(' ') << std::setw(width) << s.straight_prev << ", "
     << std::setfill(' ') << std::setw(width) << s.straight_post << ", "
     << std::setfill(' ') << std::setw(width) << s.v_ref << ", ";
  os << std::setfill(' ') << std::setw(6) << s.m_dddth << ", "
     << std::setfill(' ') << std::setw(6) << s.m_ddth << ", "
     << std::setfill(' ') << std::setw(6) << s.m_dth;
  os << ");";
  os << std::endl;
}

void printDefinitions() {
  printDefinition(std::cout, "SS_S90L ", SS_S90L);
  printDefinition(std::cout, "SS_S90R ", SS_S90R);
  printDefinition(std::cout, "SS_F45L ", SS_F45L);
  printDefinition(std::cout, "SS_F45R ", SS_F45R);
  printDefinition(std::cout, "SS_F90L ", SS_F90L);
  printDefinition(std::cout, "SS_F90R ", SS_F90R);
  printDefinition(std::cout, "SS_F135L", SS_F135L);
  printDefinition(std::cout, "SS_F135R", SS_F135R);
  printDefinition(std::cout, "SS_F180L", SS_F180L);
  printDefinition(std::cout, "SS_F180R", SS_F180R);
  printDefinition(std::cout, "SS_FV90L", SS_FV90L);
  printDefinition(std::cout, "SS_FV90R", SS_FV90R);
  printDefinition(std::cout, "SS_FS90L", SS_FS90L);
  printDefinition(std::cout, "SS_FS90R", SS_FS90R);
  printDefinition(std::cout, "SS_FK90L", SS_FK90L);
  printDefinition(std::cout, "SS_FK90R", SS_FK90R);
}

void printCsv(const std::string &filebase, const slalom::Shape &ss,
              const float th_start = 0) {
  auto st = slalom::Trajectory(ss);
  const float v = 600;
  State s;
  st.reset(v, th_start, ss.straight_prev / v);
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
  const std::vector<float> ticks = {{
      st.getAccelDesigner().t_0(),
      st.getAccelDesigner().t_1(),
      st.getAccelDesigner().t_2(),
      st.getAccelDesigner().t_3(),
      st.getAccelDesigner().t_3() + ss.straight_post / v,
  }};
  float t = 0;
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
  // printCsv("shape/shape_0", SS_S90R);
  // printCsv("shape/shape_1", SS_F45R);
  // printCsv("shape/shape_2", SS_F90R);
  // printCsv("shape/shape_3", SS_F135R);
  // printCsv("shape/shape_4", SS_F180R);
  // printCsv("shape/shape_5", SS_FV90R, -M_PI / 4);
  // printCsv("shape/shape_6", SS_FK90R, -M_PI / 4);
  printCsv("shape/shape_0", SS_S90L);
  printCsv("shape/shape_1", SS_F45L);
  printCsv("shape/shape_2", SS_F90L);
  printCsv("shape/shape_3", SS_F135L);
  printCsv("shape/shape_4", SS_F180L);
  printCsv("shape/shape_5", SS_FV90L, M_PI / 4);
  printCsv("shape/shape_6", SS_FK90L, M_PI / 4);

  return 0;
}
