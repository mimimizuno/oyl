#!/usr/bin/env bash

# 出力ディレクトリ作成
mkdir -p output
# 出力ファイルのリセット
rm -f output/*

set -e  # どこかでエラーが出たら即終了

BUILD_DIR=build
EXECUTABLE=MainApp.exe
TESTING=OFF
JOBS=8  # 並列スレッド数（適宜変更）

# 初回CMake構成のみ
if [ ! -f ${BUILD_DIR}/Makefile ]; then
  cmake -S . -B ${BUILD_DIR} -G "MinGW Makefiles" -DBUILD_TESTING=${TESTING}
fi

# ビルド（並列化）
cmake --build ${BUILD_DIR} --parallel ${JOBS}

# 実行
./${BUILD_DIR}/${EXECUTABLE}