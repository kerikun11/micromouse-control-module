/**
 * @file trajectory_tracker.h
 * @brief 独立2輪車の線形化フィードバック軌道追従コントローラ
 * @author Ryotaro Onuki <kerikun11+github@gmail.com>
 * @date 2019-03-31
 * @copyright Copyright 2019 Ryotaro Onuki <kerikun11+github@gmail.com>
 * @see
 * https://www.researchgate.net/publication/321620382_Ramsete_Articulated_and_Mobile_Robotics_for_Services_and_Technologies
 */
#pragma once

#include "polar.h"
#include "pose.h"
#include "state.h"

/**
 * @brief 制御関係の名前空間
 */
namespace ctrl {

/**
 * @brief 独立2輪車の軌道追従フィードバック制御器
 */
class TrajectoryTracker {
 public:
  /**
   * @brief 制御周期（積分周期）のデフォルト値 [s]
   */
  static constexpr const float kIntegrationPeriodDefault = 1e-3f;
  /**
   * @brief 制御則の切り替え閾値のデフォルト値 [mm/s]
   */
  static constexpr const float kXiThresholdDefault = 150.0f;
  /**
   * @brief フィードバックゲインを格納する構造体
   */
  struct Gain {
    float zeta = 1.0f;
    float omega_n = 15.0f;
    float low_zeta = 1.0f;  //< zeta \in [0,1]
    float low_b = 1e-3f;    //< b > 0
  };
  /**
   * @brief 計算結果を格納する構造体
   */
  struct Result {
    float v;   //**< @brief 並進速度 [mm/s]
    float w;   //**< @brief 角速度 [rad/s]
    float dv;  //**< @brief 並進加速度 [mm/s/s]
    float dw;  //**< @brief 角加速度 [rad/s/s]
  };
  /**
   * @brief 自作の sinc 関数 sinc(x) := sin(x) / x
   *
   * @param[in] x
   * @return sinc(x)
   */
  static constexpr float sinc(const float x) {
    const auto xx = x * x;
    const auto xxxx = xx * xx;
    return xxxx * xxxx / 362880 - xxxx * xx / 5040 + xxxx / 120 - xx / 6 + 1;
  }

 public:
  /**
   * @brief コンストラクタ
   *
   * @param[in] gain 軌道追従フィードバックゲイン
   * @param[in] xi_threshold 制御則を切り替える閾値
   */
  TrajectoryTracker(const Gain& gain,
                    const float xi_threshold = kXiThresholdDefault)
      : gain(gain), xi_threshold(xi_threshold) {}
  /**
   * @brief 状態の初期化
   *
   * @param[in] vs 初期並進速度
   */
  void reset(const float vs = 0) { xi = vs; }
  /**
   * @brief 制御入力の計算
   *
   * @param[in] est_q 推定位置
   * @param[in] est_v 推定速度
   * @param[in] est_a 推定加速度
   * @param[in] ref_s 目標状態
   * @param[in] Ts 制御周期
   * @return u 制御入力
   */
  const Result update(const Pose& est_q, const Polar& est_v, const Polar& est_a,
                      const State& ref_s,
                      const float Ts = kIntegrationPeriodDefault) {
    return update(est_q, est_v, est_a, ref_s.q, ref_s.dq, ref_s.ddq, ref_s.dddq,
                  Ts);
  }
  /**
   * @brief 制御入力の計算
   *
   * @param[in] est_q 推定位置
   * @param[in] est_v 推定速度
   * @param[in] est_a 推定加速度
   * @param[in] ref_q 目標位置
   * @param[in] ref_dq 目標速度
   * @param[in] ref_ddq 目標加速度
   * @param[in] ref_dddq 目標躍度
   * @param[in] Ts 制御周期
   * @return u 制御入力
   */
  const Result update(const Pose& est_q, const Polar& est_v, const Polar& est_a,
                      const Pose& ref_q, const Pose& ref_dq,
                      const Pose& ref_ddq, const Pose& ref_dddq,
                      const float Ts = kIntegrationPeriodDefault) {
    /* Prepare Variable */
    const float x = est_q.x;
    const float y = est_q.y;
    const float theta = est_q.th;
    const float cos_theta = std::cos(theta);
    const float sin_theta = std::sin(theta);
    const float dx = est_v.tra * cos_theta;
    const float dy = est_v.tra * sin_theta;
    const float ddx = est_a.tra * cos_theta;
    const float ddy = est_a.tra * sin_theta;
    /* Feedback Gain Design */
    const float zeta = gain.zeta;
    const float omega_n = gain.omega_n;
    const float kx = omega_n * omega_n;
    const float kdx = 2 * zeta * omega_n;
    const float ky = kx;
    const float kdy = kdx;
    /* Determine Reference */
    const float dddx_r = ref_dddq.x;
    const float dddy_r = ref_dddq.y;
    const float ddx_r = ref_ddq.x;
    const float ddy_r = ref_ddq.y;
    const float dx_r = ref_dq.x;
    const float dy_r = ref_dq.y;
    const float x_r = ref_q.x;
    const float y_r = ref_q.y;
    const float th_r = ref_q.th;
    const float cos_th_r = std::cos(th_r);
    const float sin_th_r = std::sin(th_r);
    const float u1 = ddx_r + kdx * (dx_r - dx) + kx * (x_r - x);
    const float u2 = ddy_r + kdy * (dy_r - dy) + ky * (y_r - y);
    const float du1 = dddx_r + kdx * (ddx_r - ddx) + kx * (dx_r - dx);
    const float du2 = dddy_r + kdy * (ddy_r - ddy) + ky * (dy_r - dy);
    const float d_xi = u1 * cos_th_r + u2 * sin_th_r;
    /* integral the state(s) */
    xi += d_xi * Ts;
    /* determine the output signal */
    Result res;
    if (std::abs(xi) < xi_threshold) {
      const auto b = gain.low_b;        //< b > 0
      const auto zeta = gain.low_zeta;  //< zeta \in [0,1]
      const auto v_d = ref_dq.x * cos_th_r + ref_dq.y * sin_th_r;
      const auto w_d = ref_dq.th;
      const auto k1 = 2 * zeta * std::sqrt(w_d * w_d + b * v_d * v_d);
      const auto k2 = b;
      const auto k3 = k1;
      const auto v = v_d * std::cos(th_r - theta) +
                     k1 * (cos_theta * (x_r - x) + sin_theta * (y_r - y));
      const auto w = w_d +
                     k2 * v_d * sinc(th_r - theta) *
                         (-sin_theta * (x_r - x) + cos_theta * (y_r - y)) +
                     k3 * (th_r - theta);
      res.v = v;
      res.w = w;
      res.dv = ref_ddq.x * cos_th_r + ref_ddq.y * sin_th_r;
      res.dw = ref_ddq.th;
      // res.v = ref_dq.x * cos_th_r + ref_dq.y * sin_th_r;
      // res.w = red_dq.th;
      // res.dv = ref_ddq.x * cos_th_r + ref_ddq.y * sin_th_r;
      // res.dw = ref_ddq.th;
    } else {
      res.v = xi;
      res.dv = d_xi;
      res.w = (u2 * cos_th_r - u1 * sin_th_r) / xi;
      res.dw = -(2 * d_xi * res.w + du1 * sin_th_r - du2 * cos_th_r) / xi;
    }
    return res;
  }

 protected:
  Gain gain;          /**< @brief フィードバックゲイン */
  float xi;           /**< @brief 補助状態変数 */
  float xi_threshold; /**< @brief 制御則を切り替える閾値 */
};

}  // namespace ctrl
