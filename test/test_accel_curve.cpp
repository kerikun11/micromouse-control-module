#include <gtest/gtest.h>

#include <ctrl/accel_curve.h>
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
    if (AccelCurve::am > 0) {
      EXPECT_LE(v0, v1);
      EXPECT_LE(v1, v2);
      EXPECT_LE(v2, v3);
    } else {
      EXPECT_GE(v0, v1);
      EXPECT_GE(v1, v2);
      EXPECT_GE(v2, v3);
    }
    /* error tolerance */
    const float e = 1e-6f;
    /* trajectory */
    const float Ts = t_end() / 1e3f;
    for (float t = -Ts * 1000; t < t_end() + Ts * 1000; t += Ts) {
      EXPECT_LE(std::abs(j(t)), jm * (1 + e));
      EXPECT_LE(std::abs(a(t)), am * (1 + e));
      /* v(t) is between vs and ve */
      EXPECT_LE(std::abs(v(t) - vs), std::abs(ve - vs));
      EXPECT_LE(std::abs(v(t) - ve), std::abs(ve - vs));
      /* da/dt == j(t) */
      EXPECT_NEAR((a(t + Ts / 2) - a(t - Ts / 2)) / Ts, j(t), jm);
      EXPECT_NEAR((v(t + Ts / 2) - v(t - Ts / 2)) / Ts, a(t), am);
      EXPECT_NEAR((x(t + Ts / 2) - x(t - Ts / 2)) / Ts, v(t), vm);
      /* da/dt < jm */
      EXPECT_LE(std::abs(a(t + Ts / 2) - a(t - Ts / 2)) / Ts, jm * (2 + e));
      EXPECT_LE(std::abs(v(t + Ts / 2) - v(t - Ts / 2)) / Ts, am * (2 + e));
    }
  }
};

TEST(AccelCurve, RandomConstraint) {
  AccelCurveTest act;
  int n = 100;
  std::mt19937 mt{std::random_device{}()};
  std::uniform_real_distribution<float> j_urd(100000, 1000000);
  std::uniform_real_distribution<float> a_urd(100, 10000);
  std::uniform_real_distribution<float> v_urd(1, 10000);
  for (int i = 0; i < n; ++i) {
    act.test(j_urd(mt), a_urd(mt), +v_urd(mt), +v_urd(mt));
    act.test(j_urd(mt), a_urd(mt), -v_urd(mt), -v_urd(mt));
  }
}

TEST(AccelCurve, GivenConstraints) {
  AccelCurveTest act;
  const std::vector<std::vector<float>> params = {
      // jm, am, vs, ve
      {100, 10, 0, 1}, {100, 10, 0, 2}, {100, 10, 1, 2},
      {100, 10, 2, 1}, {100, 10, 2, 0}, {100, 10, 1, 0},
  };
  for (const auto &ps : params) {
    act.test(ps[0], ps[1], +ps[2], +ps[3]);
    act.test(ps[0], ps[1], -ps[2], -ps[3]);
  }
}
