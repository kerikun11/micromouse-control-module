/**
 * @file AccelDesigner.h
 * @author KERI
 * @date 2018.04.29 created
 * @date 2019.03.19 modified
 * @url https://kerikeri.top/posts/2018-04-29-accel-designer4/
 */
#pragma once

#include <algorithm> //< for std::max, std::min
#include <cmath>     //< for std::sqrt, std::cbrt, std::pow
#include <complex>   //< for std::complex
#include <fstream>   //< for std::ofstream
#include <iostream>  //< for std::cout

/* このファイルに定義されているクラス一覧 */
class AccelCurve;
class AccelDesigner;

/**
 * @class 加速曲線を生成するクラス
 * @brief 引数に従って加速曲線を生成する
 */
class AccelCurve {
public:
  /**
   * @brief 最大躍度の定数 [mm/s/s/s]
   */
  static constexpr const float j_max = 500000;

public:
  /**
   * @brief 初期化付きのコンストラクタ．
   * @param a_max   最大加速度 [mm/s/s]
   * @param v_start 始点速度   [mm/s]
   * @param v_end   終点速度   [mm/s]
   */
  AccelCurve(const float a_max, const float v_start, const float v_end) {
    reset(a_max, v_start, v_end);
  }
  /**
   * @brief 空のコンストラクタ．あとで reset() により初期化すること．
   */
  AccelCurve() {
    am = t0 = t1 = t2 = t3 = v0 = v1 = v2 = v3 = x0 = x1 = x2 = x3 = tc = tm =
        0;
  }
  /**
   * @brief 引数の拘束条件から曲線を生成する．
   * この関数によって，すべての変数が初期化される．(漏れはない)
   * @param a_max   最大加速度 [mm/s/s]
   * @param v_start 始点速度 [mm/s]
   * @param v_end   終点速度 [mm/s]
   */
  void reset(const float a_max, const float v_start, const float v_end) {
    tc = calcTimeCurve(a_max); //< 速度が曲線である時間を取得
    am = (v_end - v_start > 0) ? a_max : -a_max; //< 最大加速度の符号を決定
    v0 = v_start;                                //< 代入
    v3 = v_end;                                  //< 代入
    t0 = 0;                   //< ここでは初期値をゼロとする
    x0 = 0;                   //< ここでは初期値はゼロとする
    tm = (v3 - v0) / am - tc; //< 等加速度直線運動の時間を決定
    /* 等加速度直線運動の有無で分岐 */
    if (tm > 0) {
      /* 速度: 曲線 -> 直線 -> 曲線 */
      t1 = t0 + tc;
      t2 = t1 + tm;
      t3 = t2 + tc;
    } else {
      /* 速度: 曲線 -> 曲線 */
      t1 = t0 + std::sqrt(tc / am * (v3 - v0)); //< 速度差から算出
      t2 = t1;
      t3 = t2 + (t1 - t0);
    }
    v1 = v(t1);
    v2 = v(t2); //< 式から求めることができる
    x1 = x(t1);
    x2 = x(t2); //< 式から求めることができる
    x3 = x0 + (v0 + v3) / 2 * (t3 - t0); //< 速度グラフの面積により
  }
  /**
   * @brief 時刻$t$における躍度$j$
   * @param t 時刻[s]
   * @return 躍度[mm/s/s/s]
   */
  float j(const float t) const {
    if (t <= t0)
      return 0;
    else if (t <= t1)
      return j_max;
    else if (t <= t2)
      return 0;
    else if (t <= t3)
      return -j_max;
    else
      return 0;
  }
  /**
   * @brief 時刻$t$における加速度$a$
   * @param t 時刻[s]
   * @return 加速度[mm/s/s]
   */
  float a(const float t) const {
    if (t <= t0)
      return 0;
    else if (t <= t1)
      return 1.0f / tc * am * (t - t0);
    else if (t <= t2)
      return am;
    else if (t <= t3)
      return -1.0f / tc * am * (t - t3);
    else
      return 0;
  }
  /**
   * @brief 時刻$t$における速度$v$
   * @param t 時刻[s]
   * @return 速度[mm/s]
   */
  float v(const float t) const {
    if (t <= t0)
      return v0;
    else if (t <= t1)
      return v0 + 0.50f / tc * am * (t - t0) * (t - t0);
    else if (t <= t2)
      return v1 + am * (t - t1);
    else if (t <= t3)
      return v3 - 0.50f / tc * am * (t - t3) * (t - t3);
    else
      return v3;
  }
  /**
   * @brief 時刻$t$における位置$x$
   * @param t 時刻[s]
   * @return 位置[mm]
   */
  float x(const float t) const {
    if (t <= t0)
      return x0 + v0 * (t - t0);
    else if (t <= t1)
      return x0 + v0 * (t - t0) + am / 6 / tc * (t - t0) * (t - t0) * (t - t0);
    else if (t <= t2)
      return x1 + v1 * (t - t1) + am / 2 * (t - t1) * (t - t1);
    else if (t <= t3)
      return x3 + v3 * (t - t3) - am / 6 / tc * (t - t3) * (t - t3) * (t - t3);
    else
      return x3 + v3 * (t - t3);
  }
  /**
   * @brief 終端定数
   */
  float t_end() const { return t3; }
  float v_end() const { return v3; }
  float x_end() const { return x3; }
  /**
   * @brief 曲線加速部分の時間を決定する関数
   * @param am 最大加速度の大きさ
   */
  static float calcTimeCurve(const float am) {
    const float tc = std::abs(am) / j_max; //< 時間を算出
    return tc;
  }
  /**
   * @brief 走行距離から達しうる終点速度を算出する関数
   * @param am 最大加速度の大きさ [mm/s/s]
   * @param vs 始点速度 [mm/s]
   * @param vt 目標速度 [mm/s]
   * @param d 走行距離 [mm]
   * @return ve 終点速度 [mm/s]
   */
  static float calcVelocityEnd(float am, const float vs, const float vt,
                               const float d) {
    /* 速度が曲線となる部分の時間を決定 */
    const float tc = AccelCurve::calcTimeCurve(am);
    /* 最大加速度の符号を決定 */
    const float j = (vt - vs > 0) ? j_max : -j_max;
    am = (vt - vs > 0) ? std::abs(am) : -std::abs(am);
    /* 等加速度直線運動の有無で分岐 */
    if (d > (2 * vs + j * tc * tc) * tc) {
      /* 曲線・直線・曲線 */
      // std::cout << "d > ***" << std::endl;
      /* 2次方程式の解の公式を解く */
      const float amtc = am * tc;
      const float D = amtc * amtc - 4 * (amtc * vs - vs * vs - 2 * am * d);
      return (-amtc + std::sqrt(D)) / 2;
    }
    /* 曲線・曲線 */
    // std::cout << "d < ***" << std::endl;
    /* 3次方程式を解いて，終点速度を算出 */
    float ve; //< 変数を用意
    const float a = vs;
    const float b = am * d * d / tc;
    const float aaa = a * a * a;
    const float c0 = 27 * (32 * aaa * b + 27 * b * b);
    const float c1 = 16 * aaa + 27 * b;
    // std::cout << "c0: " << c0 << std::endl;
    // std::cout << "c1: " << c1 << std::endl;
    // if (b > 0) {
    //   std::cout << "0 < b" << std::endl;
    // } else if (b > -aaa / 2) {
    //   std::cout << "-aaa/2 < b < 0" << std::endl;
    // } else if (b > -aaa) {
    //   std::cout << "-aaa < b < -aaa/2" << std::endl;
    // } else {
    //   std::cout << "b < -aaa" << std::endl;
    // }
    if (c0 >= 0) {
      /* ルートの中が非負のとき，つまり，b >= 0 のとき */
      const float c2 = std::cbrt((std::sqrt(c0) + c1) / 2);
      ve = (c2 + 4 * a * a / c2 - a) / 3; //< 3次方程式の解
      // std::cout << "c0 >= 0, c2: " << c2 << std::endl;
    } else {
      /* ルートの中が負のとき，つまり，b < 0 のとき */
      const auto c2 =
          std::pow(std::complex<float>(c1 / 2, std::sqrt(-c0) / 2), 1.0f / 3);
      ve = (c2.real() * 2 - a) / 3; //< 3次方程式の解
      // std::cout << "c0 < 0, c2: " << c2 << std::endl;
    }
    return ve;
  }
  /** @function calcVelocityMax
   *   @brief 走行距離から最大速度を算出する関数
   *   @param am 最大加速度の大きさ [mm/s/s]
   *   @param vs 始点速度 [mm/s]
   *   @param va 飽和速度 [mm/s]
   *   @param ve 終点速度 [mm/s]
   *   @param d 走行距離 [mm]
   *   @return vm 最大速度 [mm/s]
   */
  static float calcVelocityMax(const float am, const float vs, const float ve,
                               const float d) {
    /* 速度が曲線となる部分の時間を決定 */
    const float tc = AccelCurve::calcTimeCurve(am);
    /* 2次方程式の解の公式を解く */
    const float amtc = am * tc;
    const float D = amtc * amtc - 2 * (vs + ve) * amtc + 4 * am * d +
                    2 * (vs * vs + ve * ve);
    if (D < 0) {
      /* なんかおかしい */
      std::cerr << "Error: AccelCurve::calcVelocityMax()" << std::endl;
      return vs;
    }
    return (-amtc + std::sqrt(D)) / 2; //< 2次方程式の解
  }
  /**
   * @brief 速度差から変位を算出する関数
   *
   * @param am 最大加速度の大きさ [mm/s/s]
   * @param vs 始点速度 [mm/s]
   * @param vt 目標速度 [mm/s]
   * @return float d 変位 [mm/s]
   */
  static float calcMinDistance(float am, const float vs, const float vt) {
    AccelCurve ac(am, vs, vt);
    return ac.x_end();
  }

private:
  float am;             //< 最大加速度 [m/s/s]
  float t0, t1, t2, t3; //< 境界点の時刻 [s]
  float v0, v1, v2, v3; //< 境界点の速度 [m/s]
  float x0, x1, x2, x3; //< 境界点の位置 [m]
  float tc;             //< 曲線加速の時間 [s]
  float tm;             //< 最大加速度の時間 [s]
};

/**
 * @class 加減速曲線を生成するクラス
 * @brief 引数に従って速度計画をし，加減速曲線を生成する
 */
class AccelDesigner {
public:
  /**
   * @param a_max 最大加速度 [mm/s/s]
   * @param v_start 始点速度 [mm/s]
   * @param v_end 終点速度 [mm/s]
   */
  AccelDesigner(const float a_max, const float v_start, const float v_sat,
                const float v_target, const float distance,
                const float x_start = 0, const float t_start = 0) {
    reset(a_max, v_start, v_sat, v_target, distance, x_start, t_start);
  }
  /**
   * @brief 空のコンストラクタ．あとで reset() により初期化すること．
   */
  AccelDesigner() { t0 = t1 = t2 = t3 = x0 = x3 = 0; }
  /**
   * @brief 引数の拘束条件から曲線を生成する．
   * この関数によって，すべての変数が初期化される．(漏れはない)
   */
  void reset(const float a_max, const float v_start, const float v_sat,
             const float v_target, float distance, const float x_start = 0,
             const float t_start = 0) {
    /* 最大速度の仮置き */
    float v_max = std::max({v_start, v_sat, v_target});
    float v_end = v_target;
    /* 走行距離が負の場合の例外処理 */
    if (distance < 0) {
      std::cerr << "Warning: distance < 0" << std::endl;
      v_end = v_max = v_start;
      distance = 0;
    }
    /* 走行距離から終点速度$v_e$を算出 */
    if (distance < AccelCurve::calcMinDistance(a_max, v_start, v_end)) {
      /* 走行距離から終点速度$v_e$を算出 */
      v_end = AccelCurve::calcVelocityEnd(a_max, v_start, v_target, distance);
      v_max = std::max(v_start, v_end);
    }
    /* 曲線を生成 */
    ac.reset(a_max, v_start, v_max); //< 加速
    dc.reset(a_max, v_max, v_end);   //< 減速
    /* 飽和速度に達しない場合の処理 */
    if (distance < ac.x_end() + dc.x_end()) {
      /* 走行距離から最大速度$v_m$を算出 */
      v_max = AccelCurve::calcVelocityMax(a_max, v_start, v_end, distance);
      v_max = std::min(v_max, v_sat);            //< 飽和速度で飽和
      v_max = std::max({v_max, v_start, v_end}); //< 無駄な減速を避ける
      ac.reset(a_max, v_start, v_max);           //< 加速
      dc.reset(a_max, v_max, v_end);             //< 減速
    }
    if (ac.x_end() + dc.x_end() > distance + 0.01f) {
      std::cerr << "Error: distance" << std::endl;
    }
    /* 各定数の算出 */
    x0 = x_start;
    x3 = x_start + distance;
    t0 = t_start;
    t1 = t0 + ac.t_end(); //< 曲線加速終了の時刻
    t2 = t0 + ac.t_end() +
         (distance - ac.x_end() - dc.x_end()) / v_max; //< 等速走行終了の時刻
    t3 = t0 + ac.t_end() + (distance - ac.x_end() - dc.x_end()) / v_max +
         dc.t_end();        //< 曲線減速終了の時刻
    const float e = 0.001f; //< 数値誤差分
    if (!(t0 <= t1 + e && t1 <= t2 + e && t2 <= t3 + e)) {
      std::cerr << "Error: Time Point" << std::endl;
      /* 入力情報の表示 */
      std::cout << "a_max: " << a_max << "\tv_start: " << v_start
                << "\tv_sat: " << v_sat << "\tv_target: " << v_target
                << "\tdistance: " << distance << std::endl;
      /* 表示 */
      std::cout << "v_start: " << v_start << "\tv_max: " << v_max
                << "\tv_end: " << v_end << std::endl;
      std::cout << "t0: " << t0 << "\tt1: " << t1 << "\tt2: " << t2
                << "\tt3: " << t3 << std::endl;
      std::cout << "x0: " << x0 << "\tx1: " << x0 + ac.x_end()
                << "\tx2: " << x0 + (distance - dc.x_end()) << "\tx3: " << x3
                << std::endl;
    }
  }
  /**
   * @brief 時刻$t$における躍度$j$
   * @param t 時刻[s]
   * @return 躍度[mm/s/s/s]
   */
  float j(const float t) const {
    if (t < t2)
      return ac.j(t - t0);
    else
      return dc.j(t - t2);
  }
  /**
   * @brief 時刻$t$における加速度$a$
   * @param t 時刻[s]
   * @return 加速度[mm/s/s]
   */
  float a(const float t) const {
    if (t < t2)
      return ac.a(t - t0);
    else
      return dc.a(t - t2);
  }
  /**
   * @brief 時刻$t$における速度$v$
   * @param t 時刻[s]
   * @return 速度[mm/s]
   */
  float v(const float t) const {
    if (t < t2)
      return ac.v(t - t0);
    else
      return dc.v(t - t2);
  }
  /**
   * @brief 時刻$t$における位置$x$
   * @param t 時刻[s]
   * @return 位置[mm]
   */
  float x(const float t) const {
    if (t < t2)
      return x0 + ac.x(t - t0);
    else
      return x3 - dc.x_end() + dc.x(t - t2);
  }
  /**
   * @brief 終端xx
   */
  float t_end() const { return t3; }
  float v_end() const { return dc.v_end(); }
  float x_end() const { return x3; }
  /**
   * @brief stdoutに軌道のcsvを出力する関数．
   */
  void printCsv(const float t_interval = 0.001f) const {
    for (float t = 0; t < t_end(); t += t_interval) {
      printf("%f,%f,%f\n", a(t), v(t), x(t));
    }
  }
  /**
   * @brief std::ofstream に軌道のcsvを出力する関数．
   */
  void printCsv(std::ofstream &of, const float t_interval = 0.001f) const {
    for (float t = t0; t < t_end(); t += t_interval) {
      of << t << "," << j(t) << "," << a(t) << "," << v(t) << "," << x(t)
         << std::endl;
    }
  }

private:
  float t0, t1, t2, t3; /**< 境界点の時刻 [s] */
  float x0, x3;         /**< 境界点の位置 [mm] */
  AccelCurve ac, dc;    /**< 曲線加速，曲線減速オブジェクト */
};
