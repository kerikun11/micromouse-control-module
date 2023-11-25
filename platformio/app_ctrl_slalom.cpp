/**
 * @brief slalom test
 * @author Ryotaro Onuki <kerikun11+github@gmail.com>
 * @date 2023-07-01
 * @copyright Copyright 2023 Ryotaro Onuki <kerikun11+github@gmail.com>
 */
#include <ctrl/slalom/trajectory.h>
#include <ctrl/trajectory_tracker.h>

#include <chrono>
#include <cmath>
#include <vector>

using namespace ctrl;

static constexpr auto pi = M_PI;
static constexpr auto sqrt_2 = std::sqrt(2);

std::vector<std::pair<std::string, slalom::Shape>> shapes = {{
    {"SS_S90 ", slalom::Shape(Pose(45, 45, pi / 2), 44)},
    {"SS_F45 ", slalom::Shape(Pose(90, 45, pi / 4), 30)},
    {"SS_F90 ", slalom::Shape(Pose(90, 90, pi / 2), 70)},
    {"SS_F135", slalom::Shape(Pose(45, 90, pi * 3 / 4), 80)},
    {"SS_F180", slalom::Shape(Pose(0, 90, pi), 90, 24)},
    {"SS_FV90", slalom::Shape(Pose(45 * sqrt_2, 45 * sqrt_2, pi / 2), 48)},
    {"SS_FS90", slalom::Shape(Pose(45, 45, pi / 2), 44)},
}};

int app_ctrl_slalom() {
  std::cout << std::endl;
  float dummy = 0;
  for (const auto shape : shapes) {
    std::chrono::nanoseconds sum{0};
    const int n = 10000;
    for (int i = 0; i < n; ++i) {
      const auto t_s = std::chrono::system_clock().now();
      slalom::Trajectory st(shape.second);
      st.reset(1200);
      const auto t_e = std::chrono::system_clock().now();
      dummy += st.getAccelDesigner().t_end();
      const auto us =
          std::chrono::duration_cast<std::chrono::nanoseconds>(t_e - t_s);
      sum += us;
    }
    std::cout << shape.first << "\t" << sum.count() / n << "\t[ns]"
              << std::endl;
  }
  std::cout << dummy << std::endl;
  return 0;
}
