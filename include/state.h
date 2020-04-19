/**
 * @file state.h
 * @author Ryotaro Onuki (kerikun11+github@gmail.com)
 * @brief 軌道制御の状態変数
 * @date 2020-04-19
 */
#pragma once

#include "pose.h"

/**
 * @brief 制御関係の名前空間
 */
namespace ctrl {

/**
 * @brief 軌道制御の状態変数
 */
struct State {
  Pose q;
  Pose dq;
  Pose ddq;
  Pose dddq;
};

}; // namespace ctrl
