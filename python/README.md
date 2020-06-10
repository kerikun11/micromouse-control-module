# Python Module

C++ライブラリをPythonから呼び出すためのサブプロジェクト。

マイクロマウスで実際に使用するのはC++コードだが，デバッグの際の可視化ではPythonが使えると便利である．

そこでPythonからC++ライブラリを呼び出すためのラッパーを作成した．

## 追加の依存パッケージ

Boost::Python を使用する．

- Boost
- Python3.8

インストールコマンド例

```sh
# Ubuntu 20.04
sudo apt install boost python
# Arch Linux
sudo pacman -S --needed boost boost-libs python
# MSYS2 MinGW 64bit
pacman -S --needed mingw-w64-x86_64-boost mingw-w64-x86_64-python
```

※ Pythonのバージョンが `3.8` でない場合は，[CMakeLists.txt](CMakeLists.txt)冒頭のPythonバージョンを書き換える．

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
