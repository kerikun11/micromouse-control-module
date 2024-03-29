/**
 * @file slalom.h
 * @brief 拘束条件からスラロームを軌道生成するライブラリ
 * @author Ryotaro Onuki <kerikun11+github@gmail.com>
 * @date 2020-04-19
 * @copyright Copyright 2020 Ryotaro Onuki <kerikun11+github@gmail.com>
 */
#pragma once

#include <ctrl/accel_designer.h>
#include <ctrl/pose.h>
#include <ctrl/state.h>

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
 * メンバー変数は互いに依存して決定されているので、
 * 個別に数値を変更することは許されない。
 * スラローム軌道を得るには slalom::Trajectory を用いる。
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
   * @brief 拘束条件からスラローム形状を生成するコンストラクタ
   *
   * @param[in] total 前後の直線を含めた移動位置姿勢 [m, m, rad]
   * @param[in] y_curve_end y軸方向(進行方向に垂直な方向)の移動距離 [m]。
   * カーブの大きさを決めるもので、形状の設計パラメータとなる
   * @param[in] x_adv x軸方向(進行方向)の前後の直線の長さ [m]。
   * 180度ターンの場合のみで使用。
   * @param[in] dddth_max 最大角躍度の大きさ [rad/s/s/s]
   * @param[in] ddth_max 最大角加速度の大きさ [rad/s/s]
   * @param[in] dth_max 最大角速度の大きさ [rad/s]
   */
  Shape(const Pose& total, const float y_curve_end, const float x_adv = 0,
        const float dddth_max = dddth_max_default,
        const float ddth_max = ddth_max_default,
        const float dth_max = dth_max_default)
      : total(total),
        dddth_max(dddth_max),
        ddth_max(ddth_max),
        dth_max(dth_max) {
    /* 生成準備 */
    const float Ts = 1.5e-3f;  //< シミュレーションの積分周期
    float v = 600.0f;          //< 初期値
    State s;                   //< シミュレーションの状態
    AccelDesigner ad;
    ad.reset(dddth_max, ddth_max, dth_max, 0, 0, total.th);
    /* 複数回行って精度を高める */
    for (int i = 0; i < 3; ++i) {
      s.q.x = s.q.y = 0;
      /* シミュレーション */
      float t = 0;
      while (t + Ts < ad.t_end()) integrate(ad, s, v, t, Ts), t += Ts;
      integrate(ad, s, v, t, ad.t_end() - t);  //< 残りの半端分を積分
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
   * @brief 生成済みスラローム形状を単に代入するコンストラクタ
   *
   * @param[in] total 前後の直線を含めた移動位置姿勢 [m, m, rad]
   * @param[in] curve 曲線部分の変位 [m, m, rad]
   * @param[in] straight_prev 曲線前の直線の長さ [m]
   * @param[in] straight_post 曲線後の直線の長さ [m]
   * @param[in] v_ref 基準並進速度 [m/s]
   * @param[in] dddth_max 最大角躍度の大きさ [rad/s/s/s]
   * @param[in] ddth_max 最大角加速度の大きさ [rad/s/s]
   * @param[in] dth_max 最大角速度の大きさ [rad/s]
   */
  Shape(const Pose& total, const Pose& curve, float straight_prev,
        const float straight_post, const float v_ref, const float dddth_max,
        const float ddth_max, const float dth_max)
      : total(total),
        curve(curve),
        straight_prev(straight_prev),
        straight_post(straight_post),
        v_ref(v_ref),
        dddth_max(dddth_max),
        ddth_max(ddth_max),
        dth_max(dth_max) {}
  /**
   * @brief 軌道の積分を行う関数。ルンゲクッタ法を使用して数値積分を行う。
   *
   * @param[in] ad 角速度分布
   * @param[inout] s 状態変数
   * @param[in] v 並進速度 [m/s]
   * @param[in] t 時刻 [s]
   * @param[in] Ts 積分時間 [s]
   * @param[in] k_slip スリップ角定数
   */
  static void integrate(const AccelDesigner& ad, State& s, const float v,
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
  friend std::ostream& operator<<(std::ostream& os, const Shape& obj) {
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

}  // namespace slalom
}  // namespace ctrl
