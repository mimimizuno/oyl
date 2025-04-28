#!/usr/bin/env bash

# 出力ディレクトリ作成
mkdir -p output
# 出力ファイルのリセット
rm output/seo.mp4
rm output/tunnel_log.txt
rm output/trrigerseo1515.txt
rm output/trrigerseo1615.txt
rm output/trrigerseo1715.txt

# ビルドフォルダに移動
cd "$(dirname "$0")/build"

# rm output/tunnelwtcalc_log.txt

# CMakeで構成（初回またはCMakeLists.txt変更時だけ）
if [ ! -f Makefile ]; then
# -DBUILD_TESTING=ONだとテスト用ビルド込み。=OFFだとテスト用のビルドはしない
  cmake -G "MinGW Makefiles"  -DBUILD_TESTING=ON .. || exit 1
fi

# makeでビルド
mingw32-make || exit 1

# 実行
./MainApp.exe
./UnitTests