#include "accel_curve.h"
#include "gtest/gtest.h"

#include <random>

using namespace ctrl;

class AccelCurveTest : public AccelCurve {
public:
  void test(const float jm, const float am, const float vs, const float ve) {
    reset(jm, am, vs, ve);
    const auto vm = std::max(std::abs(vs), std::abs(ve));
    /* point */
    EXPECT_GE(t_end(), 0);
    EXPECT_FLOAT_EQ(v_end(), ve);
    EXPECT_LE(t0, t1);
    EXPECT_LE(t1, t2);
    EXPECT_LE(t2, t3);
    /* trajectory */
    const auto Ts = t_end() / 1e4;
    for (float t = -Ts * 1000; t < t_end() + Ts * 1000; t += Ts) {
      EXPECT_LE(std::abs(j(t)), jm * (1 + 1e-3));
      EXPECT_LE(std::abs(a(t)), am * (1 + 1e-3));
      /* vs < v(t) < ve or vs > v(t) > ve */
      EXPECT_LE(std::abs(v(t) - vs), std::abs(ve - vs));
      EXPECT_LE(std::abs(v(t) - ve), std::abs(ve - vs));
      /* da/dt == j(t) */
      EXPECT_NEAR((a(t + Ts / 2) - a(t - Ts / 2)) / Ts, j(t), jm);
      EXPECT_NEAR((v(t + Ts / 2) - v(t - Ts / 2)) / Ts, a(t), am);
      EXPECT_NEAR((x(t + Ts / 2) - x(t - Ts / 2)) / Ts, v(t), vm);
      /* da/dt < jm */
      //   EXPECT_LE(std::abs(a(t + Ts / 2) - a(t - Ts / 2)) / Ts, jm * 1.05f);
      //   EXPECT_LE(std::abs(v(t + Ts / 2) - v(t - Ts / 2)) / Ts, am * 1.05f);
    }
  }
};

TEST(AccelCurve, AccelCurve) {
  AccelCurveTest act;
  int n = 1000;
  std::mt19937 mt{std::random_device{}()};
  std::uniform_real_distribution<float> j_urd(100000, 1000000);
  std::uniform_real_distribution<float> a_urd(100, 18000);
  std::uniform_real_distribution<float> v_urd(-4800, 4800);
  for (int i = 0; i < n; ++i)
    act.test(j_urd(mt), a_urd(mt), v_urd(mt), v_urd(mt));
  act.test(240000, 9000, 0, 1200);
  act.test(240000, 900, 0, 1200);
  act.test(240000, 90, 0, 1200);
  act.test(240000, 90, 1200, 1200);
}
