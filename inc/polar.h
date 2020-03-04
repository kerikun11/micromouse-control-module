#pragma once

#include <cmath>

namespace ctrl {

class Polar {
public:
  float tra; //< translation [mm]
  float rot; //< rotation [rad]

public:
  constexpr Polar(const float tra = 0, const float rot = 0)
      : tra(tra), rot(rot) {}
  constexpr Polar(const Polar &o) : tra(o.tra), rot(o.rot) {}
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
  void clear() { tra = rot = 0; }
};

} // namespace ctrl
