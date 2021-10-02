## コンピュータでの使用例

コマンドラインでの使用例を示す．

--------------------------------------------------------------------------------

### 動作確認済みの環境

- Linux
  - Ubuntu 20.04
  - Manjaro Linux 20.0.3
- Windows
  - [MSYS2 MinGW 64 bit](https://www.msys2.org/)

--------------------------------------------------------------------------------

### 必要なパッケージ

- 必須
  - git
  - gcc, g++
  - make
  - cmake
- オプション
  - 可視化のために必要
    - python3
    - python3-matplotlib
  - Pythonモジュール化のために必要
    - pybind11
  - リファレンスの自動生成のために必要
    - doxygen
    - graphviz
  - ユニットテストのために必要
    - gtest
  - カバレッジテストのために必要
    - lcov

--------------------------------------------------------------------------------

### インストールコマンドの例

```sh
# Ubuntu 20.04
apt install git make cmake gcc g++ \
    python3-matplotlib \
    python3-dev \
    python3-distutils \
    python3-pybind11 \
    doxygen graphviz \
    libgtest-dev lcov
# Arch Linux
yay -S --needed git make cmake gcc \
    python-matplotlib \
    pybind11 \
    doxygen graphviz \
    gtest lcov
# MSYS2 MinGW 64bit
pacman -S --needed git make \
    $MINGW_PACKAGE_PREFIX-cmake \
    $MINGW_PACKAGE_PREFIX-toolchain \
    $MINGW_PACKAGE_PREFIX-python-matplotlib \
    $MINGW_PACKAGE_PREFIX-pybind11 \
    $MINGW_PACKAGE_PREFIX-doxygen \
    $MINGW_PACKAGE_PREFIX-graphviz \
    $MINGW_PACKAGE_PREFIX-gtest \
    $MINGW_PACKAGE_PREFIX-lcov
```

--------------------------------------------------------------------------------

### リポジトリの取得

このリポジトリは [CMake](https://cmake.org/) プロジェクトになっている．

はじめに以下のコマンドで初期化する．

```sh
# GitHub から clone
git clone https://github.com/kerikun11/micromouse-control-module.git
# 移動
cd micromouse-control-module
# 作業ディレクトリを作成
mkdir build
cd build
# 初期化 (Makefile の生成); MSYS2 の場合 -G"MSYS Makefiles" オプションを付加
cmake .. ${MSYSTEM:+-G"MSYS Makefiles"}
# ビルド
make
```

以降，コマンド `make` は，この `build` ディレクトリで実行すること．

--------------------------------------------------------------------------------

### 曲線加速軌道の生成とプロット

[examples/accel/main.cpp](/examples/accel/main.cpp) の実行

```sh
# 軌道 (accel_x.csv) の生成とプロット
make accel accel_plot
```

--------------------------------------------------------------------------------

### スラローム軌道の生成とプロット

[examples/slalom/main.cpp](/examples/slalom/main.cpp) の実行

```sh
# スラローム軌道 (slalom_x.csv) の生成とプロット
make slalom slalom_plot
```

--------------------------------------------------------------------------------

### Pythonモジュールの生成とプロットスクリプトの実行

C++で実装されたPythonモジュール `ctrl` を使用してプロットする．

```sh
# Python モジュール ctrl の生成とプロットスクリプトの実行
make ctrl pybind11_plot # calls ./pybind11/plot.py
```

詳しくは[こちら](/pybind11)

--------------------------------------------------------------------------------

### リファレンスの生成

コード中のコメントは [Doxygen](http://www.doxygen.jp/) に準拠しているので，API リファレンスを自動生成することができる．

```sh
# ドキュメントの自動生成
make docs
# ブラウザで開く (open コマンドは環境依存)
open docs/html/index.html
```

上記コマンドにより `build/docs/html/index.html` にリファレンスが生成される．

--------------------------------------------------------------------------------

### ユニットテスト

[GoogleTest](https://github.com/google/googletest) によるユニットテストと [LCOV](https://github.com/linux-test-project/lcov) によるカバレッジテストを実行する

```sh
# カバレッジ結果の初期化
make lcov_init
# テストを実行
make test_run
# カバレッジ結果の収集
make lcov
# ブラウザでカバレッジ結果をみる (open コマンドは環境依存)
open test/html/index.html
```

上記コマンドにより `build/test/html/index.html` にカバレッジ結果が生成される．
