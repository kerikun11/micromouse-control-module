/**
 * @file polar.h
 * @author Ryotaro Onuki (kerikun11+github@gmail.com)
 * @brief 並進と回転の座標を管理するクラスをもつファイル
 * @date 2020-04-19
 */
#pragma once

#include <cmath>

namespace ctrl {

/**
 * @brief 並進と回転の座標
 */
struct Polar {
  float tra; //< translation [m]
  float rot; //< rotation [rad]

public:
  constexpr Polar(const float tra = 0, const float rot = 0)
      : tra(tra), rot(rot) {}
  constexpr Polar(const Polar &o) : tra(o.tra), rot(o.rot) {}
  void clear() { tra = rot = 0; }
  const Polar &operator=(const Polar &o) {
    tra = o.tra;
    rot = o.rot;
    return *this;
  }
  const Polar &operator+=(const Polar &o) {
    tra += o.tra;
    rot += o.rot;
    return *this;
  }
  const Polar &operator*=(const Polar &o) {
    tra *= o.tra;
    rot *= o.rot;
    return *this;
  }
  const Polar &operator/=(const Polar &o) {
    tra /= o.tra;
    rot /= o.rot;
    return *this;
  }
  const Polar operator+(const Polar &o) const {
    return Polar(tra + o.tra, rot + o.rot);
  }
  const Polar operator-(const Polar &o) const {
    return Polar(tra - o.tra, rot - o.rot);
  }
  const Polar operator*(const Polar &o) const {
    return Polar(tra * o.tra, rot * o.rot);
  }
  const Polar operator/(const Polar &o) const {
    return Polar(tra / o.tra, rot / o.rot);
  }
  const Polar operator+(const float &k) const {
    return Polar(tra + k, rot + k);
  }
  const Polar operator-(const float &k) const {
    return Polar(tra - k, rot - k);
  }
  const Polar operator*(const float &k) const {
    return Polar(tra * k, rot * k);
  }
  const Polar operator/(const float &k) const {
    return Polar(tra / k, rot / k);
  }
};

} // namespace ctrl
