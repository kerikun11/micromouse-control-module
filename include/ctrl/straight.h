/**
 * @file straight.h
 * @author Ryotaro Onuki (kerikun11+github@gmail.com)
 * @brief 直線軌道を生成する
 * @date 2020-04-19
 */
#pragma once

#include "accel_designer.h"
#include "state.h"

/**
 * @brief 制御関係の名前空間
 */
namespace ctrl {

/**
 * @brief 直線関係の名前空間
 */
namespace straight {

/**
 * @brief straight::Trajectory 直線の軌道生成器
 *
 * ctrl::TrajectoryTracker のために用意されたクラス
 */
class Trajectory : public AccelDesigner {
public:
  /**
   * @brief 空のコンストラクタ．
   * 基底クラスの AccelDesigner::reset() により初期化すること．
   */
  Trajectory() {}
  /**
   * @brief 状態の更新
   *
   * @param s 状態変数
   * @param t 現在時刻
   */
  void update(struct State &s, const float t) const {
    s.q = Pose(x(t), 0, 0);
    s.dq = Pose(v(t), 0, 0);
    s.ddq = Pose(a(t), 0, 0);
    s.dddq = Pose(j(t), 0, 0);
  }
};

} // namespace straight
} // namespace ctrl
