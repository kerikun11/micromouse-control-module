#pragma once

#include <cmath>
#include <ostream>

namespace ctrl {

class Position {
public:
  float x, y, th;

public:
  Position(const float x = 0, const float y = 0, const float th = 0)
      : x(x), y(y), th(th) {}
  Position(const Position &o) : x(o.x), y(o.y), th(o.th) {}
  Position(const float pos[3]) : x(pos[0]), y(pos[1]), th(pos[2]) {}
  void clear() { x = y = th = 0; }
  const Position homogeneous(const Position offset) const {
    return offset + this->rotate(offset.th);
  }
  const Position rotate(const float angle) const {
    const float cos_angle = std::cos(angle);
    const float sin_angle = std::sin(angle);
    return Position(x * cos_angle - y * sin_angle,
                    x * sin_angle + y * cos_angle, th);
  }
  float getNorm() const { return std::sqrt(x * x + y * y); }
  const Position mirror_x() const { return Position(x, -y, -th); }
  bool operator==(const Position &obj) const {
    return x == obj.x && y == obj.y && th == obj.th;
  }
  const Position &operator=(const Position &o) {
    x = o.x, y = o.y, th = o.th;
    return *this;
  }
  const Position &operator-=(const Position &o) {
    x -= o.x, y -= o.y, th -= o.th;
    return *this;
  }
  const Position &operator+=(const Position &o) {
    x += o.x, y += o.y, th += o.th;
    return *this;
  }
  const Position operator+() const { return Position(x, y, th); }
  const Position operator-() const { return Position(-x, -y, -th); }
  const Position operator+(const Position &o) const {
    return Position(x + o.x, y + o.y, th + o.th);
  }
  const Position operator-(const Position &o) const {
    return Position(x - o.x, y - o.y, th - o.th);
  }
  const Position operator*(const float &k) const {
    return Position(x * k, y * k, th * k);
  }
  const Position operator/(const float &k) const {
    return Position(x / k, y / k, th / k);
  }
  friend std::ostream &operator<<(std::ostream &os, const Position &o) {
    os << "(" << o.x << ", " << o.y << ", " << o.th << ")";
    return os;
  }
};

} // namespace ctrl
