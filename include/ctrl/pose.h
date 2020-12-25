/**
 * @file pose.h
 * @author Ryotaro Onuki (kerikun11+github@gmail.com)
 * @brief 平面上の位置姿勢の座標もつファイル．
 * @date 2020-04-19
 */
#pragma once

#include <cmath>
#include <ostream>

namespace ctrl {

/**
 * @brief 位置姿勢の座標
 */
struct Pose {
  float x;  /**< @brief x 成分 [m] */
  float y;  /**< @brief y 成分 [m] */
  float th; /**< @brief theta 成分 [rad] */

public:
  Pose(const float x = 0, const float y = 0, const float th = 0)
      : x(x), y(y), th(th) {}
  void clear() { x = y = th = 0; }
  Pose mirror_x() const { return Pose(x, -y, -th); }
  Pose rotate(const float angle) const {
    const float cos_angle = std::cos(angle);
    const float sin_angle = std::sin(angle);
    return {x * cos_angle - y * sin_angle, x * sin_angle + y * cos_angle, th};
  }
  Pose homogeneous(const Pose &offset) const {
    return offset + this->rotate(offset.th);
  }
  Pose &operator+=(const Pose &o) {
    return x += o.x, y += o.y, th += o.th, *this;
  }
  Pose &operator-=(const Pose &o) {
    return x -= o.x, y -= o.y, th -= o.th, *this;
  }
  Pose operator+(const Pose &o) const { return {x + o.x, y + o.y, th + o.th}; }
  Pose operator-(const Pose &o) const { return {x - o.x, y - o.y, th - o.th}; }
  friend std::ostream &operator<<(std::ostream &os, const Pose &o) {
    return os << "(" << o.x << ", " << o.y << ", " << o.th << ")";
  }
};

} // namespace ctrl
