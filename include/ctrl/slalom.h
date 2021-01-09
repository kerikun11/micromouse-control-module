/**
 * @file slalom.h
 * @author Ryotaro Onuki (kerikun11+github@gmail.com)
 * @brief 拘束条件からスラロームを軌道生成するライブラリ
 * @date 2020-04-19
 */
#pragma once

#include "accel_designer.h"
#include "pose.h"
#include "state.h"

#include <array>
#include <cmath>
#include <ostream>

/**
 * @brief 制御関係の名前空間
 */
namespace ctrl {

/**
 * @brief スラローム関係の名前空間
 */
namespace slalom {

/**
 * @brief 最大角躍度のデフォルト値 [rad/s/s/s]
 */
static constexpr float dddth_max_default = 1200 * M_PI;
/**
 * @brief 最大角加速度のデフォルト値 [rad/s/s]
 */
static constexpr float ddth_max_default = 36 * M_PI;
/**
 * @brief 最大角速度のデフォルト値 [rad/s]
 */
static constexpr float dth_max_default = 3 * M_PI;

/**
 * @brief slalom::Shape スラロームの形状を表す構造体
 *
 * メンバー変数は互いに依存して決定されるので，個別に数値を変更することは許されない，
 * スラローム軌道を得るには slalom::Trajectory を用いる．
 */
struct Shape {
  Pose total; /**< @brief 前後の直線を含めた移動位置姿勢 */
  Pose curve; /**< @brief カーブ部分の移動位置姿勢 */
  float straight_prev; /**< @brief カーブ前の直線の距離 [m] */
  float straight_post; /**< @brief カーブ後の直線の距離 [m] */
  float v_ref;         /**< @brief カーブ部分の基準速度 [m/s] */
  float dddth_max;     /**< @brief 最大角躍度の大きさ [rad/s/s/s] */
  float ddth_max;      /**< @brief 最大角加速度の大きさ [rad/s/s] */
  float dth_max;       /**< @brief 最大角速度の大きさ [rad/s] */

public:
  /**
   * @brief 生成済みスラローム形状を単に代入するコンストラクタ
   */
  Shape(const Pose &total, const Pose &curve, float straight_prev,
        const float straight_post, const float v_ref, const float dddth_max,
        const float ddth_max, const float dth_max)
      : total(total), curve(curve), straight_prev(straight_prev),
        straight_post(straight_post), v_ref(v_ref), dddth_max(dddth_max),
        ddth_max(ddth_max), dth_max(dth_max) {}
  /**
   * @brief 拘束条件からスラローム形状を生成するコンストラクタ
   *
   * @param total 前後の直線を含めた移動位置姿勢
   * @param y_curve_end y軸方向(進行方向に垂直な方向)の移動距離，
   * カーブの大きさを決めるもので，設計パラメータとなる
   * @param x_adv x軸方向(進行方向)の前後の直線の長さ．180度ターンなどでは
   * y_curve_end で調節できないので，例外的にこの値で調節する．
   * @param dddth_max 最大角躍度の大きさ [rad/s/s/s]
   * @param ddth_max 最大角加速度の大きさ [rad/s/s]
   * @param dth_max 最大角速度の大きさ [rad/s]
   */
  Shape(const Pose &total, const float y_curve_end, const float x_adv = 0,
        const float dddth_max = dddth_max_default,
        const float ddth_max = ddth_max_default,
        const float dth_max = dth_max_default)
      : total(total), dddth_max(dddth_max), ddth_max(ddth_max),
        dth_max(dth_max) {
    /* 生成準備 */
    const float Ts = 1.5e-3f; /*< シミュレーションの積分周期 */
    float v = 600.0f;         /*< 初期値 */
    State s;                  /*< シミュレーションの状態 */
    AccelDesigner ad;
    ad.reset(dddth_max, ddth_max, dth_max, 0, 0, total.th);
    /* 複数回行って精度を高める */
    for (int i = 0; i < 3; ++i) {
      s.q.x = s.q.y = 0;
      /* シミュレーション */
      float t = 0;
      while (t + Ts < ad.t_end())
        integrate(ad, s, v, t, Ts), t += Ts;
      integrate(ad, s, v, t, ad.t_end() - t); //< 残りの半端分を積分
      /* 結果を用いて更新 */
      v *= y_curve_end / s.q.y;
    }
    curve = s.q;
    v_ref = v;
    const float sin_th = std::sin(total.th);
    const float cos_th = std::cos(total.th);
    /* 前後の直線の長さを決定 */
    if (std::abs(sin_th) < 1e-3f) {
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
  /**
   * @brief 軌道の積分を行う関数．ルンゲクッタ法を使用して数値積分を行う．
   *
   * @param ad 角速度分布
   * @param s 状態変数
   * @param v 並進速度 [m/s]
   * @param t 時刻 [s]
   * @param Ts 積分時間 [s]
   * @param k_slip スリップ角定数
   */
  static void integrate(const AccelDesigner &ad, State &s, const float v,
                        const float t, const float Ts, const float k_slip = 0) {
    /* Calculation */
    const std::array<float, 3> th{{ad.x(t), ad.x(t + Ts / 2), ad.x(t + Ts)}};
    const std::array<float, 3> w{{ad.v(t), ad.v(t + Ts / 2), ad.v(t + Ts)}};
    std::array<float, 3> cos_th;
    std::array<float, 3> sin_th;
    for (int i = 0; i < 3; ++i) {
      const auto th_slip = std::atan(-k_slip * v * w[i]);
      cos_th[i] = std::cos(th[i] + th_slip);
      sin_th[i] = std::sin(th[i] + th_slip);
    }
    /* Runge-Kutta Integral */
    s.q.x += v * Ts * (cos_th[0] + 4 * cos_th[1] + cos_th[2]) / 6;
    s.q.y += v * Ts * (sin_th[0] + 4 * sin_th[1] + sin_th[2]) / 6;
    /* Result */
    s.dq.x = v * cos_th[2];
    s.dq.y = v * sin_th[2];
    s.q.th = ad.x(t + Ts);
    s.dq.th = ad.v(t + Ts);
    s.ddq.th = ad.a(t + Ts);
    s.dddq.th = ad.j(t + Ts);
    s.ddq.x = -s.dq.y * s.dq.th;
    s.ddq.y = +s.dq.x * s.dq.th;
    s.dddq.x = -s.ddq.y * s.dq.th - s.dq.y * s.ddq.th;
    s.dddq.y = +s.ddq.x * s.dq.th + s.dq.x * s.ddq.th;
  }
  /**
   * @brief 情報の表示
   */
  friend std::ostream &operator<<(std::ostream &os, const Shape &obj) {
    os << "Slalom Shape" << std::endl;
    os << "\ttotal:\t" << obj.total << std::endl;
    os << "\tcurve:\t" << obj.curve << std::endl;
    os << "\tv_ref:\t" << obj.v_ref << std::endl;
    os << "\tstraight_prev:\t" << obj.straight_prev << std::endl;
    os << "\tstraight_post:\t" << obj.straight_post << std::endl;
    auto end = Pose(obj.straight_prev) + obj.curve +
               Pose(obj.straight_post).rotate(obj.curve.th);
    os << "\tintegral error:\t" << obj.total - end << std::endl;
    return os;
  }
};

/**
 * @brief slalom::Trajectory スラローム軌道を生成するクラス
 *
 * スラローム形状 Shape と並進速度をもとに，各時刻における位置や速度を提供する．
 */
class Trajectory {
public:
  /**
   * @brief コンストラクタ
   *
   * @param shape スラローム形状
   * @param mirror_x スラローム形状を$x$軸反転(進行方向に対して左右反転)する
   */
  Trajectory(const Shape &shape, const bool mirror_x = false) : shape(shape) {
    if (mirror_x) {
      this->shape.curve = shape.curve.mirror_x();
      this->shape.total = shape.total.mirror_x();
    }
  }
  /**
   * @brief 並進速度を設定して軌道を初期化する関数
   *
   * @param velocity 並進速度 [m/s]
   * @param th_start 初期姿勢 [rad] (オプション)
   * @param t_start 初期時刻 [s] (オプション)
   */
  void reset(const float velocity, const float th_start = 0,
             const float t_start = 0) {
    this->velocity = velocity;
    const float gain = velocity / shape.v_ref;
    ad.reset(gain * gain * gain * shape.dddth_max, gain * gain * shape.ddth_max,
             gain * shape.dth_max, 0, 0, shape.total.th, th_start, t_start);
  }
  /**
   * @brief 軌道の更新
   *
   * @param state 次の時刻に更新する現在状態
   * @param t 現在時刻 [s]
   * @param Ts 積分時間 [s]
   * @param k_slip スリップ角の比例定数
   */
  void update(State &state, const float t, const float Ts,
              const float k_slip = 0) const {
    return Shape::integrate(ad, state, velocity, t, Ts, k_slip);
  }
  /**
   * @brief 並進速度を取得
   */
  float getVelocity() const { return velocity; }
  /**
   * @brief ターンの合計時間を取得
   */
  float getTimeCurve() const { return ad.t_end(); }
  /**
   * @brief スラローム形状を取得
   */
  const Shape &getShape() const { return shape; }
  /**
   * @brief 角速度設計器を取得
   */
  const AccelDesigner &getAccelDesigner() const { return ad; }

protected:
  Shape shape;      /**< @brief スラロームの形状 */
  AccelDesigner ad; /**< @brief 角速度用の曲線加速生成器 */
  float velocity;   /**< @brief 並進速度 */
};

} // namespace slalom
} // namespace ctrl
