/**
 * @file accumulator.h
 * @author Ryotaro Onuki (kerikun11+github@gmail.com)
 * @brief リングバッファにより一定数のデータを蓄積するクラスを定義
 * @date 2019-02-02
 */
#pragma once

#include <new>

namespace ctrl {

/**
 * @brief データの蓄積器
 *
 * @tparam T データの型
 * @tparam S 蓄積するデータの数
 */
template <typename T, std::size_t S> class Accumulator {
public:
  /**
   * @brief コンストラクタ
   *
   * @param value バッファ内の全データに代入する初期値
   */
  Accumulator(const T &value = T()) {
    buffer = new T[S];
    head = 0;
    clear(value);
  }
  /**
   * @brief デストラクタ
   */
  ~Accumulator() { delete[] buffer; }
  /**
   * @brief バッファをクリアする関数
   *
   * @param value 代入する値
   */
  void clear(const T &value = T()) {
    for (int i = 0; i < S; i++)
      buffer[i] = value;
  }
  /**
   * @brief 最新のデータを追加する関数
   */
  void push(const T &value) {
    head = (head + 1) % S;
    buffer[head] = value;
  }
  /**
   * @brief 直近 index 番目の値を取得するオペレータ
   *
   * [0] 番目が最新のデータ，[size() - 1] 番目が最古のデータ
   *
   * @param index 直近何番目のデータかを指すインデックス
   * @return const T&
   */
  const T &operator[](const std::size_t index) const {
    return buffer[((int)S + head - index) % S];
  }
  /**
   * @brief 直近 n 個の平均を取得する関数
   *
   * @param n 平均個数
   * @return const T 平均値
   */
  const T average(const int n = S) const {
    T sum = T();
    for (int i = 0; i < n; i++) {
      sum += buffer[((int)S + head - i) % S];
    }
    return sum / n;
  }
  /**
   * @brief リングバッファのサイズを返す関数
   */
  std::size_t size() const { return S; }

private:
  T *buffer; /**< @brief リングバッファとして使う配列のポインタ */
  std::size_t head; /**< @brief リングバッファの先頭インデックス */
};

} // namespace ctrl
