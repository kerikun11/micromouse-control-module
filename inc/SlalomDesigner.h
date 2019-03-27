#pragma once

#include "AccelDesigner.h"

#include <array>

namespace signal_processing {
class SlalomDesigner {
public:
  static constexpr float m_dddth = 9600 * M_PI;
  static constexpr float m_ddth = 90 * M_PI;
  static constexpr float m_dth = 4 * M_PI;
  static constexpr float v_ref = 600.0f;

public:
  struct Constraint {
    float th;
    float x, y;
    float x_total, y_total;
    float x_prev;
    float straight_prev;
    float straight_post;
    Constraint(const float th, const float y, const float x_total,
               const float y_total, const float x_prev = 0.0f)
        : th(th), y(y), x_total(x_total), y_total(y_total), x_prev(x_prev) {}
  };
  struct State {
    float t;
    float x, dx, ddx, dddx;
    float y, dy, ddy, dddy;
    float th, dth, ddth, dddth;
    State()
        : t(0), x(0), dx(0), ddx(0), dddx(0), y(0), dy(0), ddy(0), dddy(0),
          th(0), dth(0), ddth(0), dddth(0) {}
  };

public:
  SlalomDesigner(struct Constraint c) : constraint(c) {
    velocity = v_ref;
    const float angle = constraint.th;
    const float Ts = 1e-6;
    gain_ref = 1.4333642721176147461f; //< 初期値は任意
    State s;
    for (int i = 0; i < 3; ++i) {
      ad.reset(gain_ref * gain_ref * gain_ref * m_dddth,
               gain_ref * gain_ref * m_ddth, 0, gain_ref * m_dth, 0, angle);
      s.t = s.x = s.y = 0;
      while (s.t < t_end()) {
        update(&s, Ts);
      }
      gain_ref *= s.y / constraint.y;
    }
    constraint.x = s.x;
    constraint.y = s.y;
    std::cout << "gain_ref: " << gain_ref << std::endl;
    const float sin_th = std::sin(constraint.th);
    const float cos_th = std::cos(constraint.th);
    if (std::abs(sin_th) < 0.001f) {
      /* 180度ターン */
      constraint.straight_prev = constraint.x_prev;
      constraint.straight_post = constraint.x_prev;
    } else {
      /* 180度ターン以外 */
      constraint.straight_prev = constraint.x_total - s.x -
                                 cos_th / sin_th * (constraint.y_total - s.y);
      constraint.straight_post = 1 / sin_th * (constraint.y_total - s.y);
    }
    std::cout << "straight_prev: " << constraint.straight_prev << std::endl;
    std::cout << "straight_post: " << constraint.straight_post << std::endl;
  }
  void reset(const float velocity) {
    this->velocity = velocity;
    const float gain = gain_ref * velocity / v_ref;
    const float angle = constraint.th;
    ad.reset(gain * gain * gain * m_dddth, gain * gain * m_ddth, 0,
             gain * m_dth, 0, angle);
  }
  void update(struct State *s, const float Ts) {
    /* Preparation */
    float &t = s->t;
    const float v = velocity;
    /* Calculation */
    const std::array<float, 3> th{ad.x(t), ad.x(t + Ts / 2), ad.x(t + Ts)};
    std::array<float, 3> cos_th;
    std::array<float, 3> sin_th;
    for (int i = 0; i < 3; ++i) {
      cos_th[i] = std::cos(th[i]);
      sin_th[i] = std::sin(th[i]);
    }
    s->x += v * Ts * (cos_th[0] + 4 * cos_th[1] + cos_th[2]) / 6;
    s->y += v * Ts * (sin_th[0] + 4 * sin_th[1] + sin_th[2]) / 6;
    s->t += Ts;
    /* Result */
    s->dx = v * cos_th[0];
    s->dy = v * sin_th[0];
    s->th = ad.x(t);
    s->dth = ad.v(t);
    s->ddth = ad.a(t);
    s->dddth = ad.j(t);
    s->ddx = -s->dy * s->dth;
    s->ddy = +s->dx * s->dth;
    s->dddx = -s->ddy * s->dth - s->dy * s->ddth;
    s->dddy = +s->ddx * s->dth + s->dx * s->ddth;
  }
  float t_end() const { return ad.t_end(); }
  float th_end() const { return constraint.th; }
  float x_end() const { return constraint.x; }
  float y_end() const { return constraint.y; }
  float straight_prev() const { return constraint.straight_prev; }
  float straight_post() const { return constraint.straight_post; }

private:
  Constraint constraint;
  signal_processing::AccelDesigner ad;
  float gain_ref;
  float velocity;
};

} // namespace signal_processing
