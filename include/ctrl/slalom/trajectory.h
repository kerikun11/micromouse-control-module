/**
 * @file trajectory.h
 * @brief 拘束条件からスラロームを軌道生成するライブラリ
 * @author Ryotaro Onuki <kerikun11+github@gmail.com>
 * @date 2020-04-19
 * @copyright Copyright 2020 <kerikun11+github@gmail.com>
 */
#pragma once

#include <ctrl/slalom/slalom.h>

/**
 * @brief 制御関係の名前空間
 */
namespace ctrl {

/**
 * @brief スラローム関係の名前空間
 */
namespace slalom {

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
   * @param[in] shape スラローム形状
   * @param[in] mirror_x スラローム形状を$x$軸反転(進行方向に対して左右反転)する
   */
  Trajectory(const Shape& shape, const bool mirror_x = false) : shape(shape) {
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
   * @param[inout] state 次の時刻に更新する現在状態
   * @param[in] t 現在時刻 [s]
   * @param[in] Ts 積分時間 [s]
   * @param[in] k_slip スリップ角の比例定数
   */
  void update(State& state, const float t, const float Ts,
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
  const Shape& getShape() const { return shape; }
  /**
   * @brief 角速度設計器を取得
   */
  const AccelDesigner& getAccelDesigner() const { return ad; }

 protected:
  Shape shape;      /**< @brief スラロームの形状 */
  AccelDesigner ad; /**< @brief 角速度用の曲線加速生成器 */
  float velocity;   /**< @brief 並進速度 */
};

}  // namespace slalom
}  // namespace ctrl
