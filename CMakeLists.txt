cmake_minimum_required(VERSION 3.10)
project(MyProject)
set(CMAKE_CXX_STANDARD 17)

# OpenCV
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# インクルードディレクトリを追加（oyl/utils.hpp を使うため）
include_directories(${PROJECT_SOURCE_DIR}/include)

# ライブラリ（src/**.cpp のビルド）
add_library(oyl-utils
 src/seo_class.cpp
 src/multi_seo_class.cpp
 src/oyl_video.cpp
)
target_include_directories(oyl-utils PUBLIC ${PROJECT_SOURCE_DIR}/include)

# main.cpp 実行ファイル
add_executable(MainApp main.cpp)
# サンプルを実行したい場合は下のように記述。↑のmain.cppはコメントアウト
# add_executable(MainApp sample/onewayunit-sample.cpp)
target_link_libraries(MainApp PRIVATE oyl-utils ${OpenCV_LIBS})

# テストオプション
option(BUILD_TESTING "Build unit tests" ON)
if (BUILD_TESTING)
    enable_testing()
    find_package(GTest REQUIRED)
    include_directories(${GTEST_INCLUDE_DIRS})

    add_executable(UnitTests
        test/test_seo_class.cpp
        test/grid_2dim_seo_test.cpp
        test/test_simulation2d_output.cpp
    )

    target_link_libraries(UnitTests
        PRIVATE
        GTest::GTest
        GTest::Main
        oyl-utils
        ${OpenCV_LIBS}
    )

    add_test(NAME AllTests COMMAND UnitTests)
endif()
