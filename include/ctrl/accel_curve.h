/**
 * @file accel_curve.h
 * @author Ryotaro Onuki (kerikun11+github@gmail.com)
 * @see https://www.kerislab.jp/posts/2018-04-29-accel-designer4/
 * @brief 躍度0次，加速度1次，速度2次，位置3次関数により，滑らかな加速を実現する
 * @date 2020-04-19
 */
#pragma once

#include <array>
#include <cmath>    //< for std::sqrt, std::cbrt
#include <iostream> //< for std::cout
#include <ostream>

/* log level definition */
#define CTRL_LOG_LEVEL_NONE 0
#define CTRL_LOG_LEVEL_ERROR 1
#define CTRL_LOG_LEVEL_WARNING 2
#define CTRL_LOG_LEVEL_INFO 3
#define CTRL_LOG_LEVEL_DEBUG 4
/* set log level */
#ifndef CTRL_LOG_LEVEL
#define CTRL_LOG_LEVEL CTRL_LOG_LEVEL_WARNING
#endif
/* Log Error */
#if CTRL_LOG_LEVEL >= CTRL_LOG_LEVEL_ERROR
#define ctrl_loge (std::cout << "[E][" __FILE__ ":" << __LINE__ << "]\t")
#else
#define ctrl_loge std::ostream(0)
#endif
/* Log Warning */
#if CTRL_LOG_LEVEL >= CTRL_LOG_LEVEL_WARNING
#define ctrl_logw (std::cout << "[W][" __FILE__ ":" << __LINE__ << "]\t")
#else
#define ctrl_logw std::ostream(0)
#endif
/* Log Info */
#if CTRL_LOG_LEVEL >= CTRL_LOG_LEVEL_INFO
#define ctrl_logi (std::cout << "[I][" __FILE__ ":" << __LINE__ << "]\t")
#else
#define ctrl_logi std::ostream(0)
#endif
/* Log Debug */
#if CTRL_LOG_LEVEL >= CTRL_LOG_LEVEL_DEBUG
#define ctrl_logd (std::cout "[D][" << __FILE__ ":" << __LINE__ << "]\t")
#else
#define ctrl_logd std::ostream(0)
#endif

/**
 * @brief 制御関係の名前空間
 */
namespace ctrl {

/**
 * @brief 加速曲線を生成するクラス
 *
 * - 引数の拘束に従って加速曲線を生成する
 * - 始点速度と終点速度を滑らかにつなぐ
 * - 移動距離の拘束はない
 * - 始点速度および終点速度は，正でも負でも可
 */
class AccelCurve {
public:
  /**
   * @brief 初期化付きのコンストラクタ．
   *
   * @param j_max   最大躍度の大きさ [m/s/s/s], 正であること
   * @param a_max   最大加速度の大きさ [m/s/s], 正であること
   * @param v_start 始点速度 [m/s]
   * @param v_end   終点速度 [m/s]
   */
  AccelCurve(const float j_max, const float a_max, const float v_start,
             const float v_end) {
    reset(j_max, a_max, v_start, v_end);
  }
  /**
   * @brief 空のコンストラクタ．あとで reset() により初期化すること．
   */
  AccelCurve() {
    jm = am = t0 = t1 = t2 = t3 = v0 = v1 = v2 = v3 = x0 = x1 = x2 = x3 = 0;
  }
  /**
   * @brief 引数の拘束条件から曲線を生成する．
   * この関数によって，すべての変数が初期化される．(漏れはない)
   *
   * @param j_max   最大躍度の大きさ [m/s/s/s], 正であること
   * @param a_max   最大加速度の大きさ [m/s/s], 正であること
   * @param v_start 始点速度 [m/s]
   * @param v_end   終点速度 [m/s]
   */
  void reset(const float j_max, const float a_max, const float v_start,
             const float v_end) {
    /* 符号付きで代入 */
    am = (v_end > v_start) ? a_max : -a_max; //< 最大加速度の符号を決定
    jm = (v_end > v_start) ? j_max : -j_max; //< 最大躍度の符号を決定
    /* 初期値と最終値を代入 */
    v0 = v_start; //< 代入
    v3 = v_end;   //< 代入
    t0 = 0;       //< ここでは初期値をゼロとする
    x0 = 0;       //< ここでは初期値はゼロとする
    /* 速度が曲線となる部分の時間を決定 */
    const auto tc = a_max / j_max;
    /* 等加速度直線運動の時間を決定 */
    const auto tm = (v3 - v0) / am - tc;
    /* 等加速度直線運動の有無で分岐 */
    if (tm > 0) {
      /* 速度: 曲線 -> 直線 -> 曲線 */
      t1 = t0 + tc;
      t2 = t1 + tm;
      t3 = t2 + tc;
      v1 = v0 + am * tc / 2;                //< v(t) を積分
      v2 = v1 + am * tm;                    //< v(t) を積分
      x1 = x0 + v0 * tc + am * tc * tc / 6; //< x(t) を積分
      x2 = x1 + v1 * tm;                    //< x(t) を積分
      x3 = x0 + (v0 + v3) / 2 * (t3 - t0); //< v(t) グラフの台形の面積より
    } else {
      /* 速度: 曲線 -> 曲線 */
      const auto tcp = std::sqrt((v3 - v0) / jm); //< 変曲までの時間
      t1 = t2 = t0 + tcp;
      t3 = t2 + tcp;
      v1 = v2 = (v0 + v3) / 2; //< 対称性より中点となる
      x1 = x2 = x0 + v1 * tcp + jm * tcp * tcp * tcp / 6; //< x(t) を積分
      x3 = x0 + 2 * v1 * tcp; //< 速度 v(t) グラフの面積より
    }
  }
  /**
   * @brief 時刻 t [s] における躍度 j [m/s/s/s]
   */
  float j(const float t) const {
    if (t <= t0)
      return 0;
    else if (t <= t1)
      return jm;
    else if (t <= t2)
      return 0;
    else if (t <= t3)
      return -jm;
    else
      return 0;
  }
  /**
   * @brief 時刻 t [s] における加速度 a [m/s/s]
   */
  float a(const float t) const {
    if (t <= t0)
      return 0;
    else if (t <= t1)
      return jm * (t - t0);
    else if (t <= t2)
      return am;
    else if (t <= t3)
      return -jm * (t - t3);
    else
      return 0;
  }
  /**
   * @brief 時刻 t [s] における速度 v [m/s]
   */
  float v(const float t) const {
    if (t <= t0)
      return v0;
    else if (t <= t1)
      return v0 + jm / 2 * (t - t0) * (t - t0);
    else if (t <= t2)
      return v1 + am * (t - t1);
    else if (t <= t3)
      return v3 - jm / 2 * (t - t3) * (t - t3);
    else
      return v3;
  }
  /**
   * @brief 時刻 t [s] における位置 x [m]
   */
  float x(const float t) const {
    if (t <= t0)
      return x0 + v0 * (t - t0);
    else if (t <= t1)
      return x0 + v0 * (t - t0) + jm / 6 * (t - t0) * (t - t0) * (t - t0);
    else if (t <= t2)
      return x1 + v1 * (t - t1) + am / 2 * (t - t1) * (t - t1);
    else if (t <= t3)
      return x3 + v3 * (t - t3) - jm / 6 * (t - t3) * (t - t3) * (t - t3);
    else
      return x3 + v3 * (t - t3);
  }
  /**
   * @brief 終点時刻 [s]
   */
  float t_end() const { return t3; }
  /**
   * @brief 終点速度 [m/s]
   */
  float v_end() const { return v3; }
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
   * @brief 境界のタイムスタンプを取得
   */
  const std::array<float, 4> getTimeStamp() const { return {{t0, t1, t2, t3}}; }
  /**
   * @brief std::ostream に軌道のcsvを出力する関数．
   */
  void printCsv(std::ostream &os, const float t_interval = 1e-3f) const {
    for (float t = t0; t < t_end(); t += t_interval) {
      os << t << "," << j(t) << "," << a(t) << "," << v(t) << "," << x(t)
         << std::endl;
    }
  }
  /**
   * @brief 情報の表示
   */
  friend std::ostream &operator<<(std::ostream &os, const AccelCurve &obj) {
    os << "AccelCurve ";
    os << "\tvs: " << obj.v0;
    os << "\tve: " << obj.v3;
    os << "\tt0: " << obj.t0;
    os << "\tt1: " << obj.t1;
    os << "\tt2: " << obj.t2;
    os << "\tt3: " << obj.t3;
    os << "\td: " << obj.x3 - obj.x0;
    return os;
  }

public:
  /**
   * @brief 走行距離から達しうる終点速度を算出する関数
   *
   * @param j_max 最大躍度の大きさ [m/s/s/s], 正であること
   * @param a_max 最大加速度の大きさ [m/s/s], 正であること
   * @param vs    始点速度 [m/s]
   * @param vt    目標速度 [m/s]
   * @param d     走行距離 [m]
   * @return ve   終点速度 [m/s]
   */
  static float calcReachableVelocityEnd(const float j_max, const float a_max,
                                        const float vs, const float vt,
                                        const float d) {
    /* 速度が曲線となる部分の時間を決定 */
    const auto tc = a_max / j_max;
    /* 最大加速度の符号を決定 */
    const auto am = (vt > vs) ? a_max : -a_max;
    const auto jm = (vt > vs) ? j_max : -j_max;
    /* 等加速度直線運動の有無で分岐 */
    const auto d_triangle = (vs + am * tc / 2) * tc; //< distance @ tm == 0
    const auto v_triangle = jm / am * d - vs;        //< v_end @ tm == 0
    // ctrl_logd << "d_tri: " << d_triangle << std::endl;
    // ctrl_logd << "v_tri: " << v_triangle << std::endl;
    if (d * v_triangle > 0 && std::abs(d) > std::abs(d_triangle)) {
      /* 曲線・直線・曲線 */
      ctrl_logd << "v: curve - straight - curve" << std::endl;
      /* 2次方程式の解の公式を解く */
      const auto amtc = am * tc;
      const auto D = amtc * amtc - 4 * (amtc * vs - vs * vs - 2 * am * d);
      const auto sqrtD = std::sqrt(D);
      return (-amtc + (d > 0 ? sqrtD : -sqrtD)) / 2;
    }
    /* 曲線・曲線 (走行距離が短すぎる) */
    /* 3次方程式を解いて，終点速度を算出;
     * 簡単のため，値を一度すべて正に変換して，計算結果に符号を付与して返送 */
    const auto a = std::abs(vs);
    const auto b = (d > 0 ? 1 : -1) * jm * d * d;
    const auto aaa_27 = a * a * a / 27;
    const auto cr = 8 * aaa_27 + b / 2;
    const auto ci_b = 8 * aaa_27 / b + 1.0f / 4;
    if (ci_b >= 0) {
      /* ルートの中が非負のとき，3乗根により解を求める */
      ctrl_logd << "v: curve - curve (accel)" << std::endl;
      const auto c = std::cbrt(cr + std::abs(b) * std::sqrt(ci_b));
      return (d > 0 ? 1 : -1) * (c + 4 * a * a / c / 9 - a / 3);
    } else {
      /* ルートの中が負のとき，極座標変換して解を求める */
      ctrl_logd << "v: curve - curve (decel)" << std::endl;
      const auto ci = std::abs(b) * std::sqrt(-ci_b);
      const auto r = std::hypot(cr, ci); //< = sqrt(cr^2 + ci^2)
      const auto th = std::atan2(ci, cr);
      return (d > 0 ? 1 : -1) * (2 * std::cbrt(r) * std::cos(th / 3) - a / 3);
    }
  }
  /**
   * @brief 走行距離から達しうる最大速度を算出する関数
   *
   * @param j_max 最大躍度の大きさ [m/s/s/s], 正であること
   * @param a_max 最大加速度の大きさ [m/s/s], 正であること
   * @param vs    始点速度 [m/s]
   * @param ve    終点速度 [m/s]
   * @param d     走行距離 [m]
   * @return vm   最大速度 [m/s]
   */
  static float calcReachableVelocityMax(const float j_max, const float a_max,
                                        const float vs, const float ve,
                                        const float d) {
    /* 速度が曲線となる部分の時間を決定 */
    const auto tc = a_max / j_max;
    const auto am = (d > 0) ? a_max : -a_max; /*< 加速方向は移動方向に依存 */
    /* 2次方程式の解の公式を解く */
    const auto amtc = am * tc;
    const auto D = amtc * amtc - 2 * (vs + ve) * amtc + 4 * am * d +
                   2 * (vs * vs + ve * ve);
    if (D < 0) {
      /* 拘束条件がおかしい */
      ctrl_loge << "Error! D = " << D << " < 0" << std::endl;
      /* 入力のチェック */
      if (vs * ve < 0)
        ctrl_loge << "Invalid Input! vs: " << vs << ", ve: " << ve << std::endl;
      return vs;
    }
    const auto sqrtD = std::sqrt(D);
    return (-amtc + (d > 0 ? sqrtD : -sqrtD)) / 2; //< 2次方程式の解
  }
  /**
   * @brief 速度差から変位を算出する関数
   *
   * @param j_max   最大躍度の大きさ [m/s/s/s], 正であること
   * @param a_max   最大加速度の大きさ [m/s/s], 正であること
   * @param v_start 始点速度 [m/s]
   * @param v_end   終点速度 [m/s]
   * @return d      変位 [m]
   */
  static float calcDistanceFromVelocityStartToEnd(const float j_max,
                                                  const float a_max,
                                                  const float v_start,
                                                  const float v_end) {
    /* 符号付きで代入 */
    const auto am = (v_end > v_start) ? a_max : -a_max;
    const auto jm = (v_end > v_start) ? j_max : -j_max;
    /* 速度が曲線となる部分の時間を決定 */
    const auto tc = a_max / j_max;
    /* 等加速度直線運動の時間を決定 */
    const auto tm = (v_end - v_start) / am - tc;
    /* 始点から終点までの時間を決定 */
    const auto t_all =
        (tm > 0) ? (tc + tm + tc) : (2 * std::sqrt((v_end - v_start) / jm));
    return (v_start + v_end) / 2 * t_all; //< 速度グラフの面積により
  }

protected:
  float jm;             /**< @brief 躍度定数 [m/s/s/s] */
  float am;             /**< @brief 加速度定数 [m/s/s] */
  float t0, t1, t2, t3; /**< @brief 時刻定数 [s] */
  float v0, v1, v2, v3; /**< @brief 速度定数 [m/s] */
  float x0, x1, x2, x3; /**< @brief 位置定数 [m] */
};
} // namespace ctrl
