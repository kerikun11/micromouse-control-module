#pragma once

#include "AccelDesigner.h"
#include "Position.h"

#include <array>
#include <ostream> //< for std::ostream

namespace ctrl {

namespace straight {

class Trajectory : public AccelDesigner {
public:
  Trajectory() {}
  void update(struct State &s, const float t) const {
    s.q = ctrl::Position(x(t), 0);
    s.dq = ctrl::Position(v(t), 0);
    s.ddq = ctrl::Position(a(t), 0);
    s.dddq = ctrl::Position(j(t), 0);
  }
};

} // namespace straight
} // namespace ctrl
