#pragma once

#include "position.h"

namespace ctrl {

struct State {
  Position q;
  Position dq;
  Position ddq;
  Position dddq;
};

}; // namespace ctrl
