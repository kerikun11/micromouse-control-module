/**
 * @file feedback_controller.h
 * @author Ryotaro Onuki (kerikun11+github@gmail.com)
 * @brief フィードバック制御器クラスを保持するファイル
 * @date 2020-04-10
 * @copyright Copyright (c) 2020 Ryotaro Onuki
 */
#pragma once

#include <cmath>

/**
 * @brief 制御関係の名前空間
 */
namespace ctrl {

/**
 * @brief 1次フィードフォワード補償付きフィードバック制御器クラス
 * @tparam T 状態変数の型
 */
template <typename T> class FeedbackController {
public:
  /**
   * @brief フィードフォワード成分に使用する1次モデル
   *
   * 使用しない場合は， $ K_1 = 0,~ T_1 = 1 $ に設定すること．
   * 伝達関数 $ y(s) = \frac{K_1}{T_1s+1} u(s) $
   */
  struct Model {
    T K1; /** 1次モデルの定常ゲイン (使用しない場合は 0 とすること) */
    T T1; /** 1次モデルの時定数 (使用しない場合は 1 とすること) */
  };
  /**
   * @brief フィードバック成分に使用するPIDゲイン
   *        使用しない成分は，0に設定すること．
   *
   * 伝達関数 $ u(s) = K_p e(s) + K_i / s e(s) + K_d s e(s) $,
   * ただし，$ e(s) := r(s) - y(s) $
   */
  struct Gain {
    T Kp; /**< フィードバック比例ゲイン */
    T Ki; /**< フィードバック積分ゲイン */
    T Kd; /**< フィードバック微分ゲイン */
  };
  /**
   * @brief 制御入力の成分内訳．
   * ゲインチューニングの際に可視化するために使用する．
   */
  struct Breakdown {
    T ff;  /**< フィードフォワード成分 */
    T fb;  /**< フィードバック成分 */
    T fbp; /**< フィードバック成分のうち比例成分 */
    T fbi; /**< フィードバック成分のうち積分成分 */
    T fbd; /**< フィードバック成分のうち微分成分 */
    T u;   /**< 成分の総和 */
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
   * @brief 状態を更新して，次の制御入力を得る関数
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
    /* integrate error */
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
