/**
 * @file straight.h
 * @author Ryotaro Onuki (kerikun11+github@gmail.com)
 * @brief 直線軌道を生成する
 * @version 0.1
 * @date 2019-03-31
 * @copyright Copyright (c) 2020
 */
#pragma once

#include "accel_designer.h"
#include "trajectory_tracker.h"

namespace ctrl {
namespace straight {

class Trajectory : public AccelDesigner {
public:
  Trajectory() {}
  void update(struct State &s, const float t) const {
    s.q = Position(x(t), 0);
    s.dq = Position(v(t), 0);
    s.ddq = Position(a(t), 0);
    s.dddq = Position(j(t), 0);
  }
};

} // namespace straight
} // namespace ctrl
