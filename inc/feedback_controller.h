/**
 * @file feedback_controller.h
 * @author Ryotaro Onuki (kerikun11+github@gmail.com)
 * @brief フィードバック制御器クラスを保持するファイル
 * @date 2020-04-01
 *
 * @copyright Copyright (c) 2020
 *
 */
#pragma once

#include <cmath>

namespace ctrl {

/**
 * @brief 1次フィードフォワード＋フィードバック制御クラス
 *
 * @tparam T 状態変数の型
 */
template <typename T> class FeedbackController {
public:
  /**
   * @brief フィードフォワード成分に使用する1次モデル
   *
   * 使用しない場合は、 $ K_1 = 0,~ T_1 = 1 $ に設定する。
   *
   * 伝達関数 $ y(s) = \frac{K_1}{T_1s+1} u(s) $
   */
  struct Model {
    T K1;
    T T1;
  };
  /**
   * @brief フィードバック成分に使用するPIDゲイン
   *
   * 使用しない成分は、0に設定する。
   *
   * 伝達関数 $ u(s) = K_p e(s) + K_i s e(s) + K_d e(s) / s $,
   * $ e(s) := r(s) - y(s) $
   */
  struct Gain {
    T Kp;
    T Ki;
    T Kd;
  };
  /**
   * @brief 制御入力の計算内訳。ゲイン設計時に使用するとよい。
   */
  struct Breakdown {
    T ff, fbp, fbi, fbd, fb, u;
  };

public:
  /**
   * @brief コンストラクタ
   *
   * @param M フィードフォワードモデル
   * @param G フィードバックゲイン
   */
  FeedbackController(const Model &M, const Gain &G) : M(M), G(G) { reset(); }
  /**
   * @brief 積分項をリセットする関数
   */
  void reset() { e_int = T(); }
  /**
   * @brief 状態を更新して、次の制御入力を得る関数
   *
   * @param r 目標値
   * @param y 観測値
   * @param dr 目標値の微分
   * @param dy 観測値の微分
   * @param Ts 離散時間周期
   */
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
  /** @brief フィードフォワードモデルを取得する関数 */
  const Model &getModel() const { return M; }
  /** @brief フィードバックゲインを取得する関数 */
  const Gain &getGain() const { return G; }
  /** @brief 制御入力の内訳を取得する関数 */
  const Breakdown &getBreakdown() const { return bd; }

protected:
  Model M;      /**< @brief フィードフォワードモデル */
  Gain G;       /**< @brief フィードバックゲイン */
  Breakdown bd; /**< @brief 制御入力の計算内訳 */
  T e_int;      /**< @brief 追従誤差の積分値 */
};

}; // namespace ctrl
