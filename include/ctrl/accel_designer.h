/**
 * @file accel_designer.h
 * @brief 距離の拘束を満たす加減速走行軌道を生成するクラスを保持するファイル
 * @author Ryotaro Onuki <kerikun11+github@gmail.com>
 * @date 2020-04-19
 * @see https://www.kerislab.jp/posts/2018-04-29-accel-designer4/
 */
#pragma once

#include "accel_curve.h"

#include <algorithm>  //< for std::max, std::min
#include <array>
#include <iostream>  //< for std::cout
#include <ostream>

/**
 * @brief 制御関係の名前空間
 */
namespace ctrl {

/**
 * @brief 拘束条件を満たす曲線加減速の軌道を生成するクラス
 *
 * - 移動距離の拘束条件を満たす曲線加速軌道を生成する
 * - 各時刻 $t$ における躍度 $j(t)$，加速度 $a(t)$，速度 $v(t)$，位置 $x(t)$
 * を提供する
 * - 最大加速度 $a_{\\max}$ と始点速度 $v_s$
 * など拘束次第では目標速度 $v_t$ に達することができない場合があるので注意する
 */
class AccelDesigner {
 public:
  /**
   * @brief 初期化付きコンストラクタ
   *
   * @param j_max     最大躍度の大きさ [m/s/s/s]，正であること
   * @param a_max     最大加速度の大きさ [m/s/s], 正であること
   * @param v_max     最大速度の大きさ [m/s]，正であること
   * @param v_start   始点速度 [m/s]
   * @param v_target  目標速度 [m/s]
   * @param dist      移動距離 [m]
   * @param x_start   始点位置 [m] (オプション)
   * @param t_start   始点時刻 [s] (オプション)
   */
  AccelDesigner(const float j_max,
                const float a_max,
                const float v_max,
                const float v_start,
                const float v_target,
                const float dist,
                const float x_start = 0,
                const float t_start = 0) {
    reset(j_max, a_max, v_max, v_start, v_target, dist, x_start, t_start);
  }
  /**
   * @brief 空のコンストラクタ．あとで reset() により初期化すること．
   */
  AccelDesigner() { t0 = t1 = t2 = t3 = x0 = x3 = 0; }
  /**
   * @brief 引数の拘束条件から曲線を生成する．
   * この関数によって，すべての変数が初期化される．(漏れはない)
   *
   * @param j_max     最大躍度の大きさ [m/s/s/s]，正であること
   * @param a_max     最大加速度の大きさ [m/s/s], 正であること
   * @param v_max     最大速度の大きさ [m/s]，正であること
   * @param v_start   始点速度 [m/s]
   * @param v_target  目標速度 [m/s]
   * @param dist      移動距離 [m]
   * @param x_start   始点位置 [m] (オプション)
   * @param t_start   始点時刻 [s] (オプション)
   */
  void reset(const float j_max,
             const float a_max,
             const float v_max,
             const float v_start,
             const float v_target,
             const float dist,
             const float x_start = 0,
             const float t_start = 0) {
    /* 目標速度に到達可能か，走行距離から終点速度を決定していく */
    auto v_end = v_target; /*< 仮代入 */
    /* 移動距離の拘束により，目標速度に達し得ない場合の処理 */
    const auto dist_min = AccelCurve::calcDistanceFromVelocityStartToEnd(
        j_max, a_max, v_start, v_end);
    if (std::abs(dist) < std::abs(dist_min)) {
      ctrl_logd << "vs -> ve != vt" << std::endl;
      /* 目標速度$v_t$に向かい，走行距離$d$で到達し得る終点速度$v_e$を算出 */
      v_end = AccelCurve::calcReachableVelocityEnd(j_max, a_max, v_start,
                                                   v_target, dist);
    }
    /* 飽和速度の仮置き */
    auto v_sat = dist > 0 ? std::max({v_start, v_max, v_end})
                          : std::min({v_start, -v_max, v_end});
    /* 曲線を生成 */
    ac.reset(j_max, a_max, v_start, v_sat);  //< 加速部分
    dc.reset(j_max, a_max, v_sat, v_end);    //< 減速部分
    /* 最大速度まで加速すると走行距離の拘束を満たさない場合の処理 */
    const auto d_sum = ac.x_end() + dc.x_end();
    if (std::abs(dist) < std::abs(d_sum)) {
      ctrl_logd << "vs -> vr -> ve" << std::endl;
      /* 走行距離などの拘束から到達可能速度を算出 */
      const auto v_rm = AccelCurve::calcReachableVelocityMax(
          j_max, a_max, v_start, v_end, dist);
      /* 無駄な減速を回避 */
      v_sat = dist > 0 ? std::max({v_start, v_rm, v_end})
                       : std::min({v_start, v_rm, v_end});
      ac.reset(j_max, a_max, v_start, v_sat);  //< 加速
      dc.reset(j_max, a_max, v_sat, v_end);    //< 減速
    }
    /* t23 = nan 回避; vs = ve = d = 0 のときに発生 */
    if (std::abs(v_sat) < std::numeric_limits<float>::epsilon())
      v_sat = 1;
    /* 各定数の算出 */
    const auto t23 = (dist - ac.x_end() - dc.x_end()) / v_sat;
    x0 = x_start;
    x3 = x_start + dist;
    t0 = t_start;
    t1 = t0 + ac.t_end();                     //< 曲線加速終了の時刻
    t2 = t0 + ac.t_end() + t23;               //< 等速走行終了の時刻
    t3 = t0 + ac.t_end() + t23 + dc.t_end();  //< 曲線減速終了の時刻
#if 0
    /* 出力のチェック */
    const auto e = 0.01f; //< 数値誤差分
    bool show_info = false;
    /* 飽和速度時間 */
    if (t23 < 0) {
      ctrl_logd << t23 << std::endl;
      show_info = true;
    }
    /* 終点速度 */
    if (std::abs(v_start - v_end) > e + std::abs(v_start - v_target)) {
      std::cerr << "Error: Velocity Target!" << std::endl;
      show_info = true;
    }
    /* 飽和速度 */
    if (std::abs(v_sat) >
        e + std::max({v_max, std::abs(v_start), std::abs(v_end)})) {
      std::cerr << "Error: Velocity Saturation!" << std::endl;
      show_info = true;
    }
    /* タイムスタンプ */
    if (!(t0 <= t1 + e && t1 <= t2 + e && t2 <= t3 + e)) {
      ctrl_loge << "Error: Time Point Relationship!" << std::endl;
      show_info = true;
    }
    /* 入力情報の表示 */
    if (show_info) {
      ctrl_loge << "Constraints:"
           << "\tj_max: " << j_max << "\ta_max: " << a_max
           << "\tv_max: " << v_max << "\tv_start: " << v_start
           << "\tv_target: " << v_target << "\tdist: " << dist << std::endl;
      ctrl_loge << "ad.reset(" << j_max << ", " << a_max << ", " << v_max << ", "
           << v_start << ", " << v_target << ", " << dist << ");" << std::endl;
      /* 表示 */
      ctrl_loge << "Time Stamp: "
           << "\tt0: " << t0 << "\tt1: " << t1 << "\tt2: " << t2
           << "\tt3: " << t3 << std::endl;
      ctrl_loge << "Position:   "
           << "\tx0: " << x0 << "\tx1: " << x0 + ac.x_end()
           << "\tx2: " << x0 + (dist - dc.x_end()) << "\tx3: " << x3
           << std::endl;
      ctrl_loge << "Velocity:   "
           << "\tv0: " << v_start << "\tv1: " << v(t1) << "\tv2: " << v(t2)
           << "\tv3: " << v_end << std::endl;
    }
#endif
  }
  /**
   * @brief 時刻 t [s] における躍度 j [m/s/s/s]
   */
  float j(const float t) const {
    if (t < t2)
      return ac.j(t - t0);
    else
      return dc.j(t - t2);
  }
  /**
   * @brief 時刻 t [s] における加速度 a [m/s/s]
   */
  float a(const float t) const {
    if (t < t2)
      return ac.a(t - t0);
    else
      return dc.a(t - t2);
  }
  /**
   * @brief 時刻 t [s] における速度 v [m/s]
   */
  float v(const float t) const {
    if (t < t2)
      return ac.v(t - t0);
    else
      return dc.v(t - t2);
  }
  /**
   * @brief 時刻 t [s] における位置 x [m]
   */
  float x(const float t) const {
    if (t < t2)
      return x0 + ac.x(t - t0);
    else
      return x3 - dc.x_end() + dc.x(t - t2);
  }
  /**
   * @brief 終点時刻 [s]
   */
  float t_end() const { return t3; }
  /**
   * @brief 終点速度 [m/s]
   */
  float v_end() const { return dc.v_end(); }
  /**
   * @brief 終点位置 [m]
   */
  float x_end() const { return x3; }
  /**
   * @brief 境界の時刻 [s]
   */
  float t_0() const { return t0; }
  float t_1() const { return t1; }
  float t_2() const { return t2; }
  float t_3() const { return t3; }
  /**
   * @brief stdout に軌道のcsvを出力する関数．
   */
  void printCsv(const float t_interval = 1e-3f) const {
    printCsv(std::cout, t_interval);
  }
  /**
   * @brief std::ostream に軌道のcsvを出力する関数．
   */
  void printCsv(std::ostream& os, const float t_interval = 1e-3f) const {
    for (float t = t0; t < t_end(); t += t_interval)
      os << t << "," << j(t) << "," << a(t) << "," << v(t) << "," << x(t)
         << std::endl;
  }
  /**
   * @brief 情報の表示
   */
  friend std::ostream& operator<<(std::ostream& os, const AccelDesigner& obj) {
    os << "AccelDesigner:";
    os << "\td: " << obj.x3 - obj.x0;
    os << "\tvs: " << obj.ac.v(0);
    os << "\tvm: " << obj.ac.v_end();
    os << "\tve: " << obj.dc.v_end();
    os << "\tt0: " << obj.t0;
    os << "\tt1: " << obj.t1;
    os << "\tt2: " << obj.t2;
    os << "\tt3: " << obj.t3;
    return os;
  }
  /**
   * @brief 境界のタイムスタンプを取得
   */
  const std::array<float, 8> getTimeStamp() const {
    return {{
        t0 + ac.t_0(),
        t0 + ac.t_1(),
        t0 + ac.t_2(),
        t0 + ac.t_3(),
        t2 + dc.t_0(),
        t2 + dc.t_1(),
        t2 + dc.t_2(),
        t2 + dc.t_3(),
    }};
  }

 protected:
  float t0, t1, t2, t3; /**< @brief 境界点の時刻 [s] */
  float x0, x3;         /**< @brief 境界点の位置 [m] */
  AccelCurve ac;        /**< @brief 曲線加速用オブジェクト */
  AccelCurve dc;        /**< @brief 曲線減速用オブジェクト */
};

}  // namespace ctrl
