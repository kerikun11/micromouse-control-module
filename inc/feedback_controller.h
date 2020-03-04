#pragma once

#include <cmath>

namespace ctrl {

template <typename T> class FeedbackController {
public:
  struct Model {
    T K1;
    T T1;
  };
  struct Gain {
    T Kp;
    T Ki;
    T Kd;
  };
  struct Breakdown {
    T ff, fbp, fbi, fbd, fb, u;
  };

public:
  FeedbackController(const Model &M, const Gain &G) : M(M), G(G) { reset(); }
  void reset() { e_int = T(); }
  const T update(const T &r, const T &y, const T &dr, const T &dy,
                 const float Ts) {
    /* feedforward signal */
    bd.ff = (M.T1 * dr + r) / M.K1;
    /* feedback signal */
    bd.fbp = G.Kp * (r - y);
    bd.fbi = G.Ki * e_int;
    bd.fbd = G.Kd * (dr - dy);
    bd.fb = bd.fbp + bd.fbi + bd.fbd;
    /* calculate control input value */
    bd.u = bd.ff + bd.fb;
    /* integral error */
    e_int += (r - y) * Ts;
    /* complete */
    return bd.u;
  }
  const Model &getModel() const { return M; }
  const Gain &getGain() const { return G; }
  const Breakdown &getBreakdown() const { return bd; }

protected:
  Model M;
  Gain G;
  Breakdown bd;
  T e_int;
};

}; // namespace ctrl
