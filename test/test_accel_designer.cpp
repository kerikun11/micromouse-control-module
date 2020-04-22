#include "gtest/gtest.h"

#include "accel_designer.h"

#include <random>

using namespace ctrl;

class AccelDesignerTest : public AccelDesigner {
public:
  void test(const float jm, const float am, const float vm, const float vs,
            const float vt, const float d, const float xs, const float ts) {
    reset(jm, am, vm, vs, vt, d, xs, ts);
    /* error tolerance */
    // const float e = 1e-6;
    /* time point relation */
    EXPECT_LE(t0, t1);
    // EXPECT_LE(t1, t2);
    EXPECT_LE(t2, t1);
    EXPECT_LE(t2, t3);
    /* distance */
    EXPECT_FLOAT_EQ(d, x3 - x0);
    /* v */
    EXPECT_LE(std::abs(vs - v_end()), std::abs(vs - vt));
    /* x */
    EXPECT_FLOAT_EQ(x(t0), xs);
    EXPECT_LE(x(t0), x(t1));
    EXPECT_LE(x(t1), x(t2));
    EXPECT_LE(x(t2), x(t3));
    EXPECT_FLOAT_EQ(x(t3), xs + d);
    /* trajectory */
  }
};

TEST(AccelDesigner, AccelDesignerTest) {
  AccelDesignerTest ad;
  int n = 1000;
  std::mt19937 mt{std::random_device{}()};
  std::uniform_real_distribution<float> j_urd(100000, 1000000);
  std::uniform_real_distribution<float> a_urd(100, 10000);
  std::uniform_real_distribution<float> v_urd(10, 10000);
  std::uniform_real_distribution<float> x_urd(1, 10000);
  std::uniform_real_distribution<float> t_urd(-100, 100);
  for (int i = 0; i < n; ++i) {
    const auto jm = j_urd(mt);
    const auto am = j_urd(mt);
    const auto vm = v_urd(mt);
    const auto vs = v_urd(mt);
    const auto vt = v_urd(mt);
    const auto xs = x_urd(mt);
    const auto ts = t_urd(mt);
    ad.reset(jm, am, vm, vs, vt, xs, ts);
    ad.reset(jm, am, vm, -vs, -vt, -xs, -ts);
  }
}
