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
#include <ostream> //< for std::ostream

namespace ctrl {

namespace slalom {

/* 走行定数 */
static constexpr float m_dddth = 1200 * M_PI;
static constexpr float m_ddth = 36 * M_PI;
static constexpr float m_dth = 3 * M_PI;

/* 状態変数 */
struct State {
  float t = 0;
  Position q;
  Position dq;
  Position ddq;
  Position dddq;
};

struct Shape {
  Position total;
  Position curve;
  float straight_prev;
  float straight_post;
  float v_ref;
  float t_ref;
  Shape(const Position total, const Position curve, float straight_prev,
        const float straight_post, const float v_ref)
      : total(total), curve(curve), straight_prev(straight_prev),
        straight_post(straight_post), v_ref(v_ref) {}
  Shape(const Position total, const float y_curve_end, const float x_adv = 0)
      : total(total) {
    const float Ts = 1e-5; /**< シミュレーションの積分周期 */
    float v = 600.0f;      /**< 初期値 */
    struct State s;        /**< シミュレーションの状態 */
    AccelDesigner ad;
    /* 複数回行って精度を高める */
    for (int i = 0; i < 2; ++i) {
      ad.reset(m_dddth, m_ddth, 0, m_dth, 0, total.th);
      s.t = s.q.x = s.q.y = 0;
      /* シミュレーション */
      while (s.t < ad.t_end())
        integrate(ad, &s, v, Ts);
      integrate(ad, &s, v, s.t - ad.t_end());
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
    /* 全体にかかる時間を算出 */
    ad.reset(m_dddth, m_ddth, 0, m_dth, 0, total.th);
    t_ref = ad.t_end() + (straight_prev + straight_post) / v_ref;
  }
  static void integrate(const AccelDesigner &ad, struct State *s, const float v,
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
    /* Runge-Kutta Integral */
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
  float getTotalTime() const { return t_ref; }
  bool operator==(const Shape &obj) const {
    return total == obj.total && curve == obj.curve &&
           straight_prev == obj.straight_prev &&
           straight_post == obj.straight_post && v_ref == obj.v_ref &&
           t_ref == obj.t_ref;
  }
  /**
   * @brief 情報の表示
   */
  std::ostream &printDefinition(std::ostream &os, std::string name) const {
    os << "static const auto " << name << " = ";
    os << "ctrl::slalom::Shape(";
    os << "ctrl::Position(" << total.x << ", " << total.y << ", " << total.th
       << "), ";
    os << "ctrl::Position(" << curve.x << ", " << curve.y << ", " << curve.th
       << "), ";
    os << straight_prev << ", " << straight_post << ", " << v_ref;
    os << "); ";
    os << "//< T: " << getTotalTime() << " [s]";
    os << std::endl;
    return os;
  }
  /**
   * @brief 情報の表示
   */
  friend std::ostream &operator<<(std::ostream &os, const Shape &obj) {
    os << "SlalomDesigner" << std::endl;
    os << "\ttotal: " << obj.total << std::endl;
    os << "\tnet: " << obj.curve << std::endl;
    os << "\tv_ref: " << obj.v_ref << std::endl;
    os << "\tstraight_prev: " << obj.straight_prev << std::endl;
    os << "\tstraight_post: " << obj.straight_post << std::endl;
    auto end = Position(obj.straight_prev) + obj.curve +
               Position(obj.straight_post).rotate(obj.curve.th);
    os << "\terror: " << obj.total - end << std::endl;
    return os;
  }
};

class Trajectory {
public:
  Trajectory(const Shape shape) : shape(shape) {}
  void reset(const float velocity) {
    this->velocity = velocity;
    const float gain = velocity / shape.v_ref;
    ad.reset(gain * gain * gain * m_dddth, gain * gain * m_ddth, 0,
             gain * m_dth, 0, shape.total.th);
  }
  void update(struct State *s, const float Ts) const {
    return Shape::integrate(ad, s, velocity, Ts);
  }
  float t_end() const { return ad.t_end(); }
  float get_v_ref() const { return shape.v_ref; }
  const Position &get_net_curve() const { return shape.curve; }
  float get_straight_prev() const { return shape.straight_prev; }
  float get_straight_post() const { return shape.straight_post; }
  const struct Shape &getShape() const { return shape; }

private:
  Shape shape;
  AccelDesigner ad;
  float velocity;
};

}; // namespace slalom
} // namespace ctrl
