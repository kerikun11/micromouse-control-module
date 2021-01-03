# MicroMouse Control Module in Python

C++ライブラリをPythonから呼び出すためのサブプロジェクト．

マイクロマウスで実際に使用するのはC++コードだが，デバッグの際の可視化ではPythonが使えると便利である．

そこでPythonからC++ライブラリを呼び出すためのラッパーを作成した．

## 追加の依存パッケージ

[必須のパッケージ](../README.md)に加えて，以下のパッケージをインストールする．

- pybind11

インストールコマンド例

```sh
# Ubuntu
sudo apt install python3-dev pybind11-dev
# Arch Linux
sudo pacman -S --needed pybind11
# MSYS2 MinGW 64bit
pacman -S --needed mingw-w64-x86_64-pybind11
```

## 概要

pybind11を用いてC++ライブラリをラッピングするPythonモジュールを定義した．

新たにC++ファイル [ctrl.cpp](ctrl.cpp) を導入して，名前空間 `ctrl` に含まれるクラスのラッパーを実装した．

このプロジェクトをビルドすると，動的ライブラリ `ctrl.so` が得られる．

これはPythonモジュールになっており，任意のPythonスクリプトから `import ctrl` することができる．

このようにして `ctrl.cpp` で定義されたC++ライブラリにPythonからアクセスすることができる．

## 実行例

[plot.py](plot.py)

```sh
# CMake ビルドディレクトリへ移動
cd ../build
# Pythonモジュール ctrl.so (または ctrl.dll) の生成
make all
# 可視化用Pythonスクリプト ./plot.py の実行
make pybind11_plot
# Pythonでプロットしたグラフが現れる
```
