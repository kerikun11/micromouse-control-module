# Python Module

C++ライブラリをPythonから呼び出すためのプロジェクト。

マイクロマウスで実際に使用するC++コードに対し、デバッグのために可視化するときPythonを使用できるようにした。

## 追加の依存パッケージ

- Boost
- Python3.8

## 概要

Boost::Pythonを用いてC++ライブラリをラッピングするPythonモジュールを定義した。

新たなC++ファイル [ctrl.cpp](ctrl.cpp) を導入して、名前空間 `ctrl` に含まれるクラスのラッパーを実装した。

このプロジェクトをビルドすると、動的ライブラリ `/build/ctrl.so` が得られる。

これはPythonモジュールになっており、任意のPythonスクリプトから `import ctrl` することができる。

このようにして `ctrl.cpp` で定義されたC++ライブラリにPythonからアクセスすることができる。

## 実行例

[plot.py](plot.py)

```sh
# CMake ビルドディレクトリへ移動
cd ../build
# Pythonモジュール ctrl.so の生成
make all
# 可視化Pythonスクリプトの実行
make python_plot
# Pythonでプロットしたグラフが現れる
```
