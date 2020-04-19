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
    am = (v_end - v_start > 0) ? a_max : -a_max; //< 最大加速度の符号を決定
    jm = (v_end - v_start > 0) ? j_max : -j_max; //< 最大躍度の符号を決定
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
   * @param d     走行距離 [m]
   * @return ve   終点速度 [m/s]
   */
  static float calcVelocityEnd(const float j_max, const float a_max,
                               const float vs, const float vt, const float d) {
    /* 速度が曲線となる部分の時間を決定 */
    const float tc = a_max / j_max;
    /* 最大加速度の符号を決定 */
    const float jm = (vt > vs) ? j_max : -j_max;
    const float am = (vt > vs) ? a_max : -a_max;
    /* 等加速度直線運動の有無で分岐 */
    if (std::abs(d) > std::abs((2 * vs + am * tc) * tc)) {
      /* 曲線・直線・曲線 */
      /* 2次方程式の解の公式を解く */
      const float amtc = am * tc;
      const float D = amtc * amtc - 4 * (amtc * vs - vs * vs - 2 * am * d);
      const float sqrtD = std::sqrt(D);
      return (-amtc + (d > 0 ? sqrtD : -sqrtD)) / 2;
    }
    /* 曲線・曲線 (走行距離が短すぎる) */
    /* 3次方程式を解いて，終点速度を算出 */
    const float a = vs;
    const float b = jm * d * d;
    const float aaa = a * a * a;
    const float c0 = 27 * (32 * aaa * b + 27 * b * b);
    const float c1 = 16 * aaa + 27 * b;
    if (c0 >= 0) {
      /* ルートの中が非負のとき，つまり，b >= 0 のとき */
      const float c2 = std::cbrt((std::sqrt(c0) + c1) / 2);
      return (c2 + 4 * a * a / c2 - a) / 3; //< 3次方程式の解
    } else {
      /* ルートの中が負のとき，つまり，b < 0 のとき */
      const auto c2 =
          std::pow(std::complex<float>(c1 / 2, std::sqrt(-c0) / 2), 1.0f / 3);
      return (c2.real() * 2 - a) / 3; //< 3次方程式の解
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
    const float tc = a_max / j_max;
    const float am = d > 0 ? a_max : -a_max;
    /* 2次方程式の解の公式を解く */
    const float amtc = am * tc;
    const float D = amtc * amtc - 2 * (vs + ve) * amtc + 4 * am * d +
                    2 * (vs * vs + ve * ve);
    if (D < 0) {
      /* なんかおかしい */
      std::cerr << "Error: AccelCurve::calcVelocityMax()" << std::endl;
      return vs;
    }
    const float sqrtD = std::sqrt(D);
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
    AccelCurve ac(j_max, a_max, v_start, v_end);
    return ac.x_end();
  }

protected:
  float jm;             /**< @brief 最大躍度 [m/s/s/s] */
  float am;             /**< @brief 最大加速度 [m/s/s] */
  float t0, t1, t2, t3; /**< @brief 境界点の時刻 [s] */
  float v0, v1, v2, v3; /**< @brief 境界点の速度 [m/s] */
  float x0, x1, x2, x3; /**< @brief 境界点の位置 [m] */
};
} // namespace ctrl
