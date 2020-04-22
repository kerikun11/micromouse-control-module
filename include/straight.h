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

class Trajectory : public AccelDesigner {
public:
  Trajectory() {}
  void update(struct State &s, const float t) const {
    s.q = Pose(x(t), 0, 0);
    s.dq = Pose(v(t), 0, 0);
    s.ddq = Pose(a(t), 0, 0);
    s.dddq = Pose(j(t), 0, 0);
  }
};

} // namespace straight
} // namespace ctrl
