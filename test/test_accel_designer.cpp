#include <gtest/gtest.h>

#include <ctrl/accel_designer.h>

#include <random>

using namespace ctrl;

class AccelDesignerTest : public AccelDesigner {
public:
  void test(const float jm, const float am, const float vm, const float vs,
            const float vt, const float d, const float xs, const float ts) {
    reset(jm, am, vm, vs, vt, d, xs, ts);
    /* error tolerance */
    const float e = 1e-5f;
    /* time point relation */
    EXPECT_FLOAT_EQ(t0, ts);
    EXPECT_LE(t0, t1 + e);
    EXPECT_LE(t1, t2 + e);
    EXPECT_LE(t2, t3 + e);
    /* velocity */
    EXPECT_FLOAT_EQ(v(t0), vs);
    EXPECT_GE(std::abs(vs - vt), std::abs(vs - v_end()));
    for (const auto &t : getTimeStamp())
      EXPECT_LE(std::abs(v(t)), std::max({vm, std::abs(vs), std::abs(vt)}));
    /* distance */
    EXPECT_NEAR(d, x3 - x0, std::abs(d) * e);
    EXPECT_NEAR(x(t0), xs, std::abs(xs) * e * 1e3f);
    EXPECT_NEAR(x(t3), xs + d, std::abs(xs + d) * e * 1e3f);
  }
};

TEST(AccelDesigner, RandomConstraints) {
  AccelDesignerTest ad;
  int n = 100;
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
    ad.test(jm, am, vm, vs, vt, xs, xs, ts);
    ad.test(jm, am, vm, -vs, -vt, -xs, -xs, -ts);
  }
}

TEST(AccelDesigner, GivenConstraints) {
  AccelDesignerTest ad;
  const std::vector<std::vector<float>> params = {
      // jm, am, vm, vs, vt, d
      {100, 10, 4, 0, 0, 0},     //< 0
      {100, 10, 4, 0, 2, 4},     //< vs -> vm -> vt, tm1>0, tm2>0
      {100, 10, 4, 0, 3, 4},     //< vs -> vm -> vt, tm1>0, tm2<0
      {100, 10, 4, 3, 0, 4},     //< vs -> vm -> vt, tm1<0, tm2>0
      {100, 10, 8, 0, 2, 4},     //< vs -> vr -> vt, vr<vm, tm1>0, tm2>0
      {100, 10, 8, 0, 6, 4},     //< vs -> vr -> vt, vr<vm, tm1>0, tm2<0
      {100, 10, 8, 0, 0.5, 0.2}, //< vs -> vr -> vt, vr<vm, tm1<0, tm2<0
      {100, 10, 6, 0, 3, 1},     //< vs -> vr -> vt, vr<vm, tm1>0, tm2<0
      {100, 10, 6, 0, 4, 1},     //< ve == vt, tm > 0 just
      {100, 10, 8, 0, 6, 1},     //< ve != vt, tm > 0, accel
      {100, 10, 8, 4, 0, 1},     //< ve != vt, tm > 0, decel
      {100, 10, 4, 0, 4, 0.1},   //< ve != vt, tm < 0, accel
      {100, 10, 4, 4, 0, 0.1},   //< ve != vt, tm < 0, decel
  };
  for (const auto &ps : params) {
    ad.test(ps[0], ps[1], ps[2], +ps[3], +ps[4], +ps[5], 0, 0);
    ad.test(ps[0], ps[1], ps[2], -ps[3], -ps[4], -ps[5], 0, 0);
  }
}
