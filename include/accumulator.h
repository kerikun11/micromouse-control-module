/**
 * @file accumulator.h
 * @author Ryotaro Onuki (kerikun11+github@gmail.com)
 * @brief リングバッファにより一定数のデータを蓄積するクラスを定義
 * @date 2019-02-02
 */
#pragma once

#include <new>

template <typename T, size_t S> class Accumulator {
public:
  Accumulator(const T &value = T()) {
    buffer = new T[S];
    head = 0;
    clear(value);
  }
  ~Accumulator() { delete buffer; }
  void clear(const T &value = T()) {
    for (int i = 0; i < S; i++)
      buffer[i] = value;
  }
  void push(const T &value) {
    head = (head + 1) % S;
    buffer[head] = value;
  }
  const T &operator[](const size_t index) const {
    return buffer[((int)S + head - index) % S];
  }
  const T average(const int num = S) const {
    T sum = T();
    for (int i = 0; i < num; i++) {
      sum += buffer[((int)S + head - i) % S];
    }
    return sum / num;
  }
  size_t size() const { return S; }

private:
  T *buffer;
  size_t head;
};
