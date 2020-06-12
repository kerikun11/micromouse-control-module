/**
 * @file main.cpp
 * @author Ryotaro Onuki (kerikun11+github@gmail.com)
 * @brief This file generates slalom shapes for each turn of the micromouse.
 * @date 2020-05-04
 */
#include <ctrl/slalom.h>

#include <filesystem>
#include <fstream>
#include <iomanip>
#include <string>
#include <vector>

using namespace ctrl;

static const float pi = M_PI;
static const float sqrt_2 = std::sqrt(2);

std::vector<std::pair<std::string, slalom::Shape>> shapes = {{
    {"SS_S90 ", slalom::Shape(Pose(45, 45, pi / 2), 44)},
    {"SS_F45 ", slalom::Shape(Pose(90, 45, pi / 4), 30)},
    {"SS_F90 ", slalom::Shape(Pose(90, 90, pi / 2), 70)},
    {"SS_F135", slalom::Shape(Pose(45, 90, pi * 3 / 4), 80)},
    {"SS_F180", slalom::Shape(Pose(0, 90, pi), 90, 24)},
    {"SS_FV90", slalom::Shape(Pose(45 * sqrt_2, 45 * sqrt_2, pi / 2), 48)},
    {"SS_FK90", slalom::Shape(Pose(90 * sqrt_2, 90 * sqrt_2, pi / 2), 125)},
    {"SS_FS90", slalom::Shape(Pose(45, 45, pi / 2), 44)},
}};

void printDefinition(std::ostream &os, const std::string &name,
                     const slalom::Shape &s) {
  const AccelDesigner ad(s.dddth_max, s.ddth_max, s.dth_max, 0, 0, s.total.th);
  const auto t_total =
      ad.t_end() + (s.straight_prev + s.straight_post) / s.v_ref;
  const auto sf = std::setfill(' ');
  os << "/* " << name << " T:" << t_total << "*/" << std::endl;
  os << "ctrl::slalom::Shape(";
  for (const auto &p : {s.total, s.curve})
    os << "ctrl::Pose(" << std::setw(8) << sf << p.x << ", " << std::setw(8)
       << sf << p.y << ", " << std::setw(9) << sf << p.th << "), ";
  os << std::setw(8) << sf << s.straight_prev << ", " << std::setw(8) << sf
     << s.straight_post << ", " << std::setw(8) << sf << s.v_ref << ", ";
  os << std::setfill(' ') << std::setw(6) << s.dddth_max << ", "
     << std::setfill(' ') << std::setw(6) << s.ddth_max << ", "
     << std::setfill(' ') << std::setw(6) << s.dth_max << "),";
  os << std::endl;
}

void printDefinitions() {
  for (const auto &[name, shape] : shapes)
    printDefinition(std::cout, name, shape);
  std::cout << std::endl;
}

void printCsv(const std::string &filebase, const slalom::Shape &ss,
              const float th_start = 0) {
  auto st = slalom::Trajectory(ss);
  const float v = 600;
  State s;
  st.reset(v, th_start, ss.straight_prev / v);
  const float Ts = 1e-5f;
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

void printTrajectory() {
  std::filesystem::create_directory("shape");
  printCsv("shape/shape_0", shapes[0].second);
  printCsv("shape/shape_1", shapes[1].second);
  printCsv("shape/shape_2", shapes[2].second);
  printCsv("shape/shape_3", shapes[3].second);
  printCsv("shape/shape_4", shapes[4].second);
  printCsv("shape/shape_5", shapes[5].second, pi / 4);
  printCsv("shape/shape_6", shapes[6].second, pi / 4);
  printCsv("shape/shape_7", shapes[7].second);
}

void printTable() {
  for (const auto &[name, shape] : shapes) {
    std::cout << "|" << name;
    std::cout << "|" << shape.total.th / float(M_PI) * 180;
    std::cout << "|(" << shape.total.x << "," << shape.total.y << ")";
    std::cout << "|(" << shape.curve.x << "," << shape.curve.y << ")";
    std::cout << "|" << shape.v_ref;
    std::cout << "|" << shape.straight_prev;
    std::cout << "|" << shape.straight_post;
    std::cout << "|" << std::endl;
  }
  std::cout << std::endl;
}

int main(void) {
  /* print definitions to stdout */
  printDefinitions();

  /* print trajectory to file*/
  printTrajectory();

  /* print result table */
  printTable();

  return 0;
}
