#!/usr/bin/env bash

# 出力ディレクトリ作成
mkdir -p output
# 出力ファイルのリセット
rm -f output/*

# ビルドフォルダに移動
cd "$(dirname "$0")/build"

# CMakeで構成（初回またはCMakeLists.txt変更時だけ）
if [ ! -f Makefile ]; then
# -DBUILD_TESTING=ONだとテスト用ビルド込み。=OFFだとテスト用のビルドはしない
  cmake -G "MinGW Makefiles"  -DBUILD_TESTING=OFF .. || exit 1
fi

# makeでビルド
mingw32-make || exit 1

# 実行
./MainApp.exe
# ./UnitTests