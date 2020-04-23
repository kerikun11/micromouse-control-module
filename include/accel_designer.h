/**
 * @file accel_designer.h
 * @author Ryotaro Onuki (kerikun11+github@gmail.com)
 * @ref https://kerikeri.top/posts/2018-04-29-accel-designer4/
 * @brief 距離の拘束を満たす加減速走行軌道を生成するクラスを保持するファイル
 * @date 2020-04-19
 */
#pragma once

#include "accel_curve.h"

#include <algorithm> //< for std::max, std::min
#include <array>
#include <iostream> //< for std::cout
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
 * - 最大加速度 $a_{\max}$ と始点速度 $v_s$
 * など拘束次第では目標速度に達することができない場合があるので注意する
 */
class AccelDesigner {
public:
  /**
   * @brief 初期化付きコンストラクタ
   *
   * @param j_max     最大躍度の大きさ [m/s/s/s]，正であること
   * @param a_max     最大加速度の大きさ [m/s/s], 正であること
   * @param v_sat     飽和速度の大きさ [m/s]，正であること
   * @param v_start   始点速度 [m/s]
   * @param v_target  目標速度 [m/s]
   * @param v_end     終点速度 [m/s]
   * @param dist      移動距離 [m]
   * @param x_start   始点位置 [m] (オプション)
   * @param t_start   始点時刻 [s] (オプション)
   */
  AccelDesigner(const float j_max, const float a_max, const float v_sat,
                const float v_start, const float v_target, const float dist,
                const float x_start = 0, const float t_start = 0) {
    reset(j_max, a_max, v_sat, v_start, v_target, dist, x_start, t_start);
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
   * @param v_sat     飽和速度の大きさ [m/s]，正であること
   * @param v_start   始点速度 [m/s]
   * @param v_target  目標速度 [m/s]
   * @param v_end     終点速度 [m/s]
   * @param dist      移動距離 [m]
   * @param x_start   始点位置 [m] (オプション)
   * @param t_start   始点時刻 [s] (オプション)
   */
  void reset(const float j_max, const float a_max, const float v_sat,
             const float v_start, const float v_target, const float dist,
             const float x_start = 0, const float t_start = 0) {
    /* 最大速度の仮置き */
    float v_max = dist > 0 ? std::max({v_start, v_sat, v_target})
                           : std::min({v_start, -v_sat, v_target});
    /* 走行距離から終点速度$v_e$を算出 */
    float v_end = v_target;
    const auto dist_min =
        AccelCurve::calcMinDistance(j_max, a_max, v_start, v_end);
    // logd << "dist_min: " << dist_min << std::endl;
    if (std::abs(dist) < std::abs(dist_min)) {
      logd << "vs -> ve != vt" << std::endl;
      /* 目標速度$v_t$に向かい，走行距離$d$で到達し得る終点速度$v_e$を算出 */
      v_end =
          AccelCurve::calcVelocityEnd(j_max, a_max, v_start, v_target, dist);
      v_max = v_end; //< 走行距離の拘束を満たすため，飽和速度まで加速できない
      // logd << "ve: " << v_end << std::endl;
    }
    /* 曲線を生成 */
    ac.reset(j_max, a_max, v_start, v_max); //< 加速
    dc.reset(j_max, a_max, v_max, v_end);   //< 減速
    /* 飽和速度まで加速すると走行距離の拘束を満たさない場合の処理 */
    const auto d_sum = ac.x_end() + dc.x_end();
    if (std::abs(dist) < std::abs(d_sum)) {
      logd << "vs -> vm -> ve" << std::endl;
      /* 走行距離から最大速度$v_m$を算出; 下記v_maxは上記v_max以下になる */
      v_max = AccelCurve::calcVelocityMax(j_max, a_max, v_start, v_end, dist);
      /* 無駄な減速を回避 */
      v_max = dist > 0 ? std::max({v_start, v_max, v_end})
                       : std::min({v_start, v_max, v_end});
      ac.reset(j_max, a_max, v_start, v_max); //< 加速
      dc.reset(j_max, a_max, v_max, v_end);   //< 減速
    }
    /* 各定数の算出 */
    x0 = x_start;
    x3 = x_start + dist;
    t0 = t_start;
    t1 = t0 + ac.t_end(); //< 曲線加速終了の時刻
    t2 = t0 + ac.t_end() +
         (dist - ac.x_end() - dc.x_end()) / v_max; //< 等速走行終了の時刻
    t3 = t0 + ac.t_end() + (dist - ac.x_end() - dc.x_end()) / v_max +
         dc.t_end(); //< 曲線減速終了の時刻
    /* 出力のチェック */
    const float e = 0.01f; //< 数値誤差分
    bool show_info = false;
    /* 終点速度 */
    if (std::abs(v_start - v_end) > e + std::abs(v_start - v_target)) {
      std::cerr << "Error: Velocity Target!" << std::endl;
      show_info = true;
    }
    /* 最大速度 */
    if (std::abs(v_max) >
        e + std::max({v_sat, std::abs(v_start), std::abs(v_end)})) {
      std::cerr << "Error: Velocity Saturation!" << std::endl;
      show_info = true;
    }
    /* タイムスタンプ */
    if (!(t0 <= t1 + e && t1 <= t2 + e && t2 <= t3 + e)) {
      loge << "Error: Time Point Relationship!" << std::endl;
      show_info = true;
    }
    /* 入力情報の表示 */
    if (show_info) {
      loge << "Constraints:"
           << "\tj_max: " << j_max << "\ta_max: " << a_max
           << "\tv_start: " << v_start << "\tv_sat: " << v_sat
           << "\tv_target: " << v_target << "\tdist: " << dist << std::endl;
      loge << "ad.reset(" << j_max << ", " << a_max << ", " << v_sat << ", "
           << v_start << ", " << v_target << ", " << dist << ");" << std::endl;
      /* 表示 */
      loge << "Time Stamp: "
           << "\tt0: " << t0 << "\tt1: " << t1 << "\tt2: " << t2
           << "\tt3: " << t3 << std::endl;
      loge << "Position:   "
           << "\tx0: " << x0 << "\tx1: " << x0 + ac.x_end()
           << "\tx2: " << x0 + (dist - dc.x_end()) << "\tx3: " << x3
           << std::endl;
      loge << "Velocity:   "
           << "\tv0: " << v_start << "\tv1: " << v(t1) << "\tv2: " << v(t2)
           << "\tv3: " << v_end << std::endl;
    }
  }
  /**
   * @brief 時刻 $t$ における躍度 $j$
   * @param t 時刻[s]
   * @return j 躍度[m/s/s/s]
   */
  float j(const float t) const {
    if (t < t2)
      return ac.j(t - t0);
    else
      return dc.j(t - t2);
  }
  /**
   * @brief 時刻 $t$ における加速度 $a$
   * @param t 時刻 [s]
   * @return a 加速度 [m/s/s]
   */
  float a(const float t) const {
    if (t < t2)
      return ac.a(t - t0);
    else
      return dc.a(t - t2);
  }
  /**
   * @brief 時刻 $t$ における速度 $v$
   * @param t 時刻 [s]
   * @return v 速度 [m/s]
   */
  float v(const float t) const {
    if (t < t2)
      return ac.v(t - t0);
    else
      return dc.v(t - t2);
  }
  /**
   * @brief 時刻 $t$ における位置 $x$
   * @param t 時刻 [s]
   * @return x 位置 [m]
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
   * @brief 境界の時刻
   */
  float t_0() const { return t0; }
  float t_1() const { return t1; }
  float t_2() const { return t2; }
  float t_3() const { return t3; }
  /**
   * @brief stdout に軌道のcsvを出力する関数．
   */
  void printCsv(const float t_interval = 0.001f) const {
    printCsv(std::cout, t_interval);
  }
  /**
   * @brief std::ostream に軌道のcsvを出力する関数．
   */
  void printCsv(std::ostream &os, const float t_interval = 0.001f) const {
    for (float t = t0; t < t_end(); t += t_interval)
      os << t << "," << j(t) << "," << a(t) << "," << v(t) << "," << x(t)
         << std::endl;
  }
  /**
   * @brief 情報の表示
   */
  friend std::ostream &operator<<(std::ostream &os, const AccelDesigner &obj) {
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
   * @return std::array<float, 8> 境界のタイムスタンプの配列
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
  AccelCurve ac, dc; /**< @brief 曲線加速，曲線減速オブジェクト */
};

} // namespace ctrl
