/**
 * @file SlalomDesigner.h
 * @author Ryotaro Onuki (kerikun11+github@gmail.com)
 * @brief 拘束条件からスラロームを軌道生成するライブラリ
 * @version 0.1
 * @date 2019-04-01
 *
 * @copyright Copyright (c) 2019
 *
 */
#pragma once

#include "AccelDesigner.h"
#include "Position.h"

#include <array>

namespace ctrl {

class SlalomDesigner {
public:
  /* 走行定数 */
  static constexpr float m_dddth = 9600 * M_PI;
  static constexpr float m_ddth = 90 * M_PI;
  static constexpr float m_dth = 4 * M_PI;

public:
  struct Shape {
    Position total;
    Position curve;
    float straight_prev;
    float straight_post;
    float v_ref;
    Shape(const Position total, const Position curve, float straight_prev,
          const float straight_post, const float v_ref)
        : total(total), curve(curve), straight_prev(straight_prev),
          straight_post(straight_post), v_ref(v_ref) {}
    Shape(const Position total, const float y_curve_end, const float x_adv = 0)
        : total(total) {
      const float Ts = 1e-5; /**< サンプリング周期 */
      float v = 600.0f;      /**< 初期値 */
      struct State s;        /**< シミュレーションの状態 */
      AccelDesigner ad;
      for (int i = 0; i < 2; ++i) {
        ad.reset(m_dddth, m_ddth, 0, m_dth, 0, total.th);
        s.t = s.q.x = s.q.y = 0;
        /* シミュレーション */
        while (s.t < ad.t_end()) {
          SlalomDesigner::update(ad, &s, v, Ts);
        }
        SlalomDesigner::update(ad, &s, v, s.t - ad.t_end());
        v *= y_curve_end / s.q.y;
      }
      curve = s.q;
      v_ref = v;
      const float sin_th = std::sin(total.th);
      const float cos_th = std::cos(total.th);
      /* 前後の直線の長さを決定 */
      if (std::abs(sin_th) < 0.001f) {
        /* 180度ターン */
        straight_prev = x_adv;
        straight_post = x_adv;
        curve = total;
      } else {
        /* 180度ターン以外 */
        straight_prev = total.x - s.q.x - cos_th / sin_th * (total.y - s.q.y);
        straight_post = 1 / sin_th * (total.y - s.q.y);
      }
    }
  };
  struct State {
    float t;
    Position q;
    Position dq;
    Position ddq;
    Position dddq;
    State() : t(0) {}
  };

public:
  SlalomDesigner(const Shape shape) : shape(shape) {}
  void reset(const float velocity) {
    this->velocity = velocity;
    const float gain = velocity / shape.v_ref;
    ad.reset(gain * gain * gain * m_dddth, gain * gain * m_ddth, 0,
             gain * m_dth, 0, shape.total.th);
  }
  void update(struct State *s, const float Ts) {
    return update(ad, s, velocity, Ts);
  }
  float t_end() const { return ad.t_end(); }
  float get_v_ref() const { return shape.v_ref; }
  const Position &get_net_curve() const { return shape.curve; }
  float get_straight_prev() const { return shape.straight_prev; }
  float get_straight_post() const { return shape.straight_post; }
  /**
   * @brief 情報の表示
   */
  friend std::ostream &operator<<(std::ostream &os, const SlalomDesigner &obj) {
    os << "SlalomDesigner" << std::endl;
    os << "\ttotal: " << obj.shape.total << std::endl;
    os << "\tnet: " << obj.shape.curve << std::endl;
    os << "\tv_ref: " << obj.shape.v_ref << std::endl;
    os << "\tstraight_prev: " << obj.shape.straight_prev << std::endl;
    os << "\tstraight_post: " << obj.shape.straight_post << std::endl;
    auto end = Position(obj.shape.straight_prev) + obj.shape.curve +
               Position(obj.shape.straight_post).rotate(obj.shape.curve.th);
    os << "\terror: " << obj.shape.total - end << std::endl;
    return os;
  }

private:
  Shape shape;
  AccelDesigner ad;
  float velocity;

  static void update(const AccelDesigner &ad, struct State *s, const float v,
                     const float Ts) {
    /* Preparation */
    float &t = s->t;
    /* Calculation */
    const std::array<float, 3> th{ad.x(t), ad.x(t + Ts / 2), ad.x(t + Ts)};
    std::array<float, 3> cos_th;
    std::array<float, 3> sin_th;
    for (int i = 0; i < 3; ++i) {
      cos_th[i] = std::cos(th[i]);
      sin_th[i] = std::sin(th[i]);
    }
    /* Integral */
    s->q.x += v * Ts * (cos_th[0] + 4 * cos_th[1] + cos_th[2]) / 6;
    s->q.y += v * Ts * (sin_th[0] + 4 * sin_th[1] + sin_th[2]) / 6;
    s->t += Ts;
    /* Result */
    s->dq.x = v * cos_th[2];
    s->dq.y = v * sin_th[2];
    s->q.th = ad.x(t);
    s->dq.th = ad.v(t);
    s->ddq.th = ad.a(t);
    s->dddq.th = ad.j(t);
    s->ddq.x = -s->dq.y * s->dq.th;
    s->ddq.y = +s->dq.x * s->dq.th;
    s->dddq.x = -s->ddq.y * s->dq.th - s->dq.y * s->ddq.th;
    s->dddq.y = +s->ddq.x * s->dq.th + s->dq.x * s->ddq.th;
  }
};

} // namespace ctrl
