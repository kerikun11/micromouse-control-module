# ライブラリ構成

## 名前空間

この制御ライブラリの実装はすべて `ctrl` 名前空間に収められている。

## クラス・構造体・共用体・型

| 型                         | 意味                 | 用途                                                 |
| -------------------------- | -------------------- | ---------------------------------------------------- |
| ctrl::AccelDesigner        | 曲線加減速設計器     | 走行距離や最大速度の拘束がある曲線加減速軌道の設計   |
| ctrl::AccelCurve           | 曲線加速設計器       | 走行距離の拘束がない単純な曲線加速軌道の設計         |
| ctrl::Polar                | 極座標               | 並進 $r$ と回転 $\theta$ の座標の管理                |
| ctrl::Pose                 | 位置姿勢座標         | 位置 $(x, y)$ と姿勢 $\theta$ の座標の管理           |
| ctrl::State                | 軌道制御の状態変数   | 位置、速度、加速度、躍度の管理                       |
| ctrl::slalom::Shape        | スラローム形状       | スラローム形状の設計                                 |
| ctrl::slalom::Trajectory   | スラローム軌道       | スラローム軌道（時間の関数）の設計                   |
| ctrl::straight::Trajectory | 直線軌道             | 直線軌道（時間の関数）の設計                         |
| ctrl::TrajectoryTracker    | 軌道追従制御器       | スラロームや直線の軌道追従制御                       |
| ctrl::FeedbackController   | フィードバック制御器 | 並進と回転速度の PID 制御                            |
| ctrl::Accumulator          | データ蓄積器         | 固定サイズのリングバッファ。サンプリングなどに使用。 |

## 定数

| 定数                                               | 意味                                 | 用途                 |
| -------------------------------------------------- | ------------------------------------ | -------------------- |
| ctrl::slalom::dddth_max_default                    | 最大角躍度のデフォルト値 [rad/s/s/s] | スラローム設計に使用 |
| ctrl::slalom::ddth_max_default                     | 最大角加速度のデフォルト値 [rad/s/s] | スラローム設計に使用 |
| ctrl::slalom::dth_max_default                      | 最大角速度のデフォルト値 [rad/s]     | スラローム設計に使用 |
| ctrl::TrajectoryTracker::kIntegrationPeriodDefault | 制御周期のデフォルト値 [s]           | 軌道追従の積分に使用 |
| ctrl::TrajectoryTracker::kXiThresholdDefault       | 制御速の切り替え閾値 [s]             | 軌道追従制御に使用   |
