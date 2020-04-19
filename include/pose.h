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
  float x, y, th; /*< (x, y, theta) 成分 */

public:
  /** デフォルトコンストラクタ */
  Pose(const float x = 0, const float y = 0, const float th = 0)
      : x(x), y(y), th(th) {}
  /** コピーコンストラクタ */
  Pose(const Pose &o) : x(o.x), y(o.y), th(o.th) {}
  void clear() { x = y = th = 0; }
  const Pose homogeneous(const Pose offset) const {
    return offset + this->rotate(offset.th);
  }
  const Pose rotate(const float angle) const {
    const float cos_angle = std::cos(angle);
    const float sin_angle = std::sin(angle);
    return Pose(x * cos_angle - y * sin_angle, x * sin_angle + y * cos_angle,
                th);
  }
  const Pose mirror_x() const { return Pose(x, -y, -th); }
  bool operator==(const Pose &obj) const {
    return x == obj.x && y == obj.y && th == obj.th;
  }
  const Pose &operator=(const Pose &o) {
    x = o.x, y = o.y, th = o.th;
    return *this;
  }
  const Pose &operator-=(const Pose &o) {
    x -= o.x, y -= o.y, th -= o.th;
    return *this;
  }
  const Pose &operator+=(const Pose &o) {
    x += o.x, y += o.y, th += o.th;
    return *this;
  }
  const Pose operator+() const { return Pose(x, y, th); }
  const Pose operator-() const { return Pose(-x, -y, -th); }
  const Pose operator+(const Pose &o) const {
    return Pose(x + o.x, y + o.y, th + o.th);
  }
  const Pose operator-(const Pose &o) const {
    return Pose(x - o.x, y - o.y, th - o.th);
  }
  const Pose operator*(const float &k) const {
    return Pose(x * k, y * k, th * k);
  }
  const Pose operator/(const float &k) const {
    return Pose(x / k, y / k, th / k);
  }
  friend std::ostream &operator<<(std::ostream &os, const Pose &o) {
    os << "(" << o.x << ", " << o.y << ", " << o.th << ")";
    return os;
  }
};

} // namespace ctrl
