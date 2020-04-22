/**
 * @file accel_curve.h
 * @author Ryotaro Onuki (kerikun11+github@gmail.com)
 * @ref https://kerikeri.top/posts/2018-04-29-accel-designer4/
 * @brief 躍度0次，加速度1次，速度2次，位置3次関数により，滑らかな加速を実現する
 * @date 2020-04-19
 */
#pragma once

#include <cmath>    //< for std::sqrt, std::cbrt, std::pow
#include <complex>  //< for std::complex
#include <iostream> //< for std::cout
#include <ostream>

/* File Path Management */
#ifdef PROJ_DIR /*< defined at CMakeLists.txt */
#include <cstring>
#include <string>
#define FILEPATH (".." + std::string(__FILE__).substr(std::strlen(PROJ_DIR)))
#else
#define FILEPATH __FILE__
#endif

/* Log Info */
#ifndef loge
#if 1
#define loge (std::cout << "[E][" << FILEPATH << ":" << __LINE__ << "]\t")
#else
#define loge std::ostream(0)
#endif
#endif
/* Log Info */
#ifndef logi
#if 0
#define logi (std::cout << "[I][" << FILEPATH << ":" << __LINE__ << "]\t")
#else
#define logi std::ostream(0)
#endif
#endif
/* Log Debug */
#ifndef logd
#if 0
#define logd (std::cout << "[D][" << FILEPATH << ":" << __LINE__ << "]\t")
#else
#define logd std::ostream(0)
#endif
#endif

/**
 * @brief 制御関係の名前空間
 */
namespace ctrl {

/**
 * @brief 加速曲線を生成するクラス
 *
 * 引数に従って加速曲線を生成する
 */
class AccelCurve {
public:
  /**
   * @brief 初期化付きのコンストラクタ．
   *
   * @param j_max   最大躍度の大きさ [m/s/s/s]
   * @param a_max   最大加速度の大きさ [m/s/s]
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
   * @param j_max   最大躍度の大きさ [m/s/s/s]
   * @param a_max   最大加速度の大きさ [m/s/s]
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
    } else {
      /* 速度: 曲線 -> 曲線 */
      t1 = t0 + std::sqrt(1 / jm * (v3 - v0)); //< 速度差から算出
      t2 = t1;             //< 加速度一定の時間はないので同じ
      t3 = t2 + (t1 - t0); //< 対称性
    }
    v2 = v1 = v0;      //< 未初期化変数の使用警告回避
    v1 = v(t1);        //< 式から求めることができる
    v2 = v(t2);        //< 式から求めることができる
    x3 = x2 = x1 = x0; //< 未初期化変数の使用警告回避
    x1 = x(t1);        //< 式から求めることができる
    x2 = x(t2);        //< 式から求めることができる
    x3 = x0 + (v0 + v3) / 2 * (t3 - t0); //< 速度グラフの面積により
  }
  /**
   * @brief 時刻 $t$ における躍度 $j$
   * @param t 時刻 [s]
   * @return j 躍度 [m/s/s/s]
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
   * @brief 時刻 $t$ における加速度 $a$
   * @param t 時刻 [s]
   * @return a 加速度 [m/s/s]
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
   * @brief 時刻 $t$ における速度 $v$
   * @param t 時刻 [s]
   * @return v 速度 [m/s]
   */
  float v(const float t) const {
    if (t <= t0)
      return v0;
    else if (t <= t1)
      return v0 + 0.50f * jm * (t - t0) * (t - t0);
    else if (t <= t2)
      return v1 + am * (t - t1);
    else if (t <= t3)
      return v3 - 0.50f * jm * (t - t3) * (t - t3);
    else
      return v3;
  }
  /**
   * @brief 時刻 $t$ における位置 $x$
   * @param t 時刻 [s]
   * @return x 位置 [m]
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
   * @brief 境界の時刻
   */
  float t_0() const { return t0; }
  float t_1() const { return t1; }
  float t_2() const { return t2; }
  float t_3() const { return t3; }
  /**
   * @brief std::ostream に軌道のcsvを出力する関数．
   */
  void printCsv(std::ostream &os, const float t_interval = 0.001f) const {
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
    os << "\tvs: " << obj.v(0);
    os << "\tve: " << obj.v_end();
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
   * @param j_max 最大躍度の大きさ [m/s/s/s]
   * @param a_max 最大加速度の大きさ [m/s/s]
   * @param vs    始点速度 [m/s]
   * @param vt    目標速度 [m/s]
   * @param d     走行距離 [m]
   * @return ve   終点速度 [m/s]
   */
  static float calcVelocityEnd(const float j_max, const float a_max,
                               const float vs, const float vt, const float d) {
    /* 速度が曲線となる部分の時間を決定 */
    const auto tc = a_max / j_max;
    /* 最大加速度の符号を決定 */
    const auto am = (vt > vs) ? a_max : -a_max;
    const auto jm = (vt > vs) ? j_max : -j_max;
    /* 等加速度直線運動の有無で分岐 */
    const auto d_triangle = (vs + am * tc / 2) * tc; //< d @ tm == 0
    const auto v_triangle = jm / am * d - vs;        //< v @ tm == 0
    logd << "d_tri: " << d_triangle << std::endl;
    logd << "v_tri: " << v_triangle << std::endl;
    if (std::abs(d) > std::abs(d_triangle) && vs * v_triangle > 0) {
      /* 曲線・直線・曲線 */
      logd << "v: curve - straight - curve" << std::endl;
      /* 2次方程式の解の公式を解く */
      const auto amtc = am * tc;
      const auto D = amtc * amtc - 4 * (amtc * vs - vs * vs - 2 * am * d);
      const auto sqrtD = std::sqrt(D);
      return (-amtc + (d > 0 ? sqrtD : -sqrtD)) / 2;
    }
    /* 曲線・曲線 (走行距離が短すぎる) */
    /* 3次方程式を解いて，終点速度を算出 */
    const auto a = std::abs(vs);
    const auto b = (d > 0 ? 1 : -1) * jm * d * d;
    const auto aaa = a * a * a;
    const auto c0 = 27 * (32 * aaa * b + 27 * b * b);
    const auto c1 = 16 * aaa + 27 * b;
    if (c0 >= 0) {
      /* ルートの中が非負のとき */
      logd << "v: curve - curve (accel)" << std::endl;
      const auto c2 = std::cbrt((std::sqrt(c0) + c1) / 2);
      return (d > 0 ? 1 : -1) * (c2 + 4 * a * a / c2 - a) / 3; //< 3次方程式の解
    } else {
      /* ルートの中が負のとき */
      logd << "v: curve - curve (decel)" << std::endl;
      const auto c2 =
          std::pow(std::complex<float>(c1 / 2, std::sqrt(-c0) / 2), 1.0f / 3);
      return (d > 0 ? 1 : -1) * (c2.real() * 2 - a) / 3; //< 3次方程式の解
    }
  }
  /**
   * @brief 走行距離から達しうる最大速度を算出する関数
   *
   * @param j_max 最大躍度の大きさ [m/s/s/s]
   * @param a_max 最大加速度の大きさ [m/s/s]
   * @param vs    始点速度 [m/s]
   * @param ve    終点速度 [m/s]
   * @param d     走行距離 [m]
   * @return vm   最大速度 [m/s]
   */
  static float calcVelocityMax(const float j_max, const float a_max,
                               const float vs, const float ve, const float d) {
    /* 速度が曲線となる部分の時間を決定 */
    const auto tc = a_max / j_max;
    const auto am = d > 0 ? a_max : -a_max; /*< 加速方向は移動方向に依存 */
    /* 2次方程式の解の公式を解く */
    const auto amtc = am * tc;
    const auto D = amtc * amtc - 2 * (vs + ve) * amtc + 4 * am * d +
                   2 * (vs * vs + ve * ve);
    if (D < 0) {
      /* 拘束条件がおかしい */
      loge << "Error! D < 0" << std::endl;
      /* 入力のチェック */
      if (vs * ve < 0)
        loge << "Invalid Input! vs: " << vs << ", ve: " << ve << std::endl;
      return vs;
    }
    const auto sqrtD = std::sqrt(D);
    return (-amtc + (d > 0 ? sqrtD : -sqrtD)) / 2; //< 2次方程式の解
  }
  /**
   * @brief 速度差から変位を算出する関数
   *
   * @param j_max   最大躍度の大きさ [m/s/s/s]
   * @param a_max   最大加速度の大きさ [m/s/s]
   * @param v_start 始点速度 [m/s]
   * @param v_end   終点速度 [m/s]
   * @return d      変位 [m]
   */
  static float calcMinDistance(const float j_max, const float a_max,
                               const float v_start, const float v_end) {
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
  float jm;             /**< @brief 最大躍度 [m/s/s/s] */
  float am;             /**< @brief 最大加速度 [m/s/s] */
  float t0, t1, t2, t3; /**< @brief 境界点の時刻 [s] */
  float v0, v1, v2, v3; /**< @brief 境界点の速度 [m/s] */
  float x0, x1, x2, x3; /**< @brief 境界点の位置 [m] */
};
} // namespace ctrl
