cmake_minimum_required(VERSION 3.16)
project(MyProject)

# C++20指定（全体に対して）
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# OpenCV
find_package(OpenCV REQUIRED)

# OpenMP
find_package(OpenMP REQUIRED)

# ソースファイルとインクルード
set(PROJECT_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/include)
file(GLOB OYL_SOURCES "${PROJECT_SOURCE_DIR}/src/*.cpp")

# ライブラリ構築
add_library(oyl-utils ${OYL_SOURCES})
target_include_directories(oyl-utils
    PUBLIC
        ${PROJECT_INCLUDE_DIR}
        ${OpenCV_INCLUDE_DIRS}
)
target_compile_features(oyl-utils PUBLIC cxx_std_20)
target_link_libraries(oyl-utils PUBLIC OpenMP::OpenMP_CXX)

# メイン実行ファイル
# メイン以外を実行したい場合は30行目を31行目のように変える
add_executable(MainApp main.cpp)
# add_executable(MainApp sample/seo-2dim-sample.cpp)

target_include_directories(MainApp
    PRIVATE
        ${PROJECT_INCLUDE_DIR}
        ${OpenCV_INCLUDE_DIRS}
)
target_compile_features(MainApp PUBLIC cxx_std_20)
target_link_libraries(MainApp PRIVATE oyl-utils ${OpenCV_LIBS} OpenMP::OpenMP_CXX)

# テスト
option(BUILD_TESTING "Build unit tests" ON)
if (BUILD_TESTING)
    enable_testing()
    find_package(GTest REQUIRED)

    file(GLOB TEST_SOURCES
        ${PROJECT_SOURCE_DIR}/test/*.cpp
    )

    add_executable(UnitTests ${TEST_SOURCES})
    target_include_directories(UnitTests PRIVATE
        ${PROJECT_INCLUDE_DIR}
        ${GTEST_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
    )
    target_compile_features(UnitTests PUBLIC cxx_std_20)

    target_link_libraries(UnitTests
        PRIVATE
            GTest::gtest
            GTest::gtest_main
            oyl-utils
            ${OpenCV_LIBS}
    )

    add_test(NAME AllTests COMMAND UnitTests)
endif()
