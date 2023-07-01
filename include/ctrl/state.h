/**
 * @file state.h
 * @brief 軌道制御の状態変数
 * @author Ryotaro Onuki <kerikun11+github@gmail.com>
 * @date 2020-04-19
 * @copyright Copyright 2020 Ryotaro Onuki <kerikun11+github@gmail.com>
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
  Pose q;     //**< @brief 位置
  Pose dq;    //**< @brief 速度
  Pose ddq;   //**< @brief 加速度
  Pose dddq;  //**< @brief 躍度
};

};  // namespace ctrl
