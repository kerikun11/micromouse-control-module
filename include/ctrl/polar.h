/**
 * @file polar.h
 * @author Ryotaro Onuki (kerikun11+github@gmail.com)
 * @brief 並進と回転の座標を管理するクラスをもつファイル
 * @date 2020-04-19
 */
#pragma once

#include <cmath>
#include <ostream>

namespace ctrl {

/**
 * @brief 並進と回転の座標
 */
struct Polar {
  float tra; //< translation [m]
  float rot; //< rotation [rad]

public:
  constexpr Polar() : tra(0), rot(0) {}
  constexpr Polar(const float tra, const float rot) : tra(tra), rot(rot) {}
  void clear() { tra = rot = 0; }
  auto &operator+=(const Polar &o) { return tra += o.tra, rot += o.rot, *this; }
  auto &operator-=(const Polar &o) { return tra -= o.tra, rot -= o.rot, *this; }
  Polar operator+(const Polar &o) const { return {tra + o.tra, rot + o.rot}; }
  Polar operator-(const Polar &o) const { return {tra - o.tra, rot - o.rot}; }
  Polar operator*(const Polar &o) const { return {tra * o.tra, rot * o.rot}; }
  Polar operator/(const Polar &o) const { return {tra / o.tra, rot / o.rot}; }
  Polar operator*(const float k) const { return {tra * k, rot * k}; }
  Polar operator/(const float k) const { return {tra / k, rot / k}; }
  friend std::ostream &operator<<(std::ostream &os, const Polar &o) {
    return os << "(" << o.tra << ", " << o.rot << ")";
  }
};

} // namespace ctrl
