// #include "gtest/gtest.h"
// #include "grid_2dim.hpp"
// #include "seo_class.hpp"

// // Grid初期化とサイズ確認
// TEST(Grid2DSEOTest, Initialization) {
//     Grid2D<SEO> grid(3, 2);
//     EXPECT_EQ(grid.numRows(), 3);
//     EXPECT_EQ(grid.numCols(), 2);

//     for (int i = 0; i < 3; ++i)
//         for (int j = 0; j < 2; ++j)
//             EXPECT_NE(grid.getElement(i, j), nullptr);
// }

// // setElement / getElement の挙動確認
// TEST(Grid2DSEOTest, ElementAccess) {
//     Grid2D<SEO> grid(2, 2);
//     auto customSEO = std::make_shared<SEO>();
//     customSEO->setVn(1.23);
//     grid.setElement(0, 0, customSEO);

//     EXPECT_DOUBLE_EQ(grid.getElement(0, 0)->getVn(), 1.23);
// }

// // ラベルと出力設定のテスト
// TEST(Grid2DSEOTest, OutputSettings) {
//     Grid2D<SEO> grid(1, 1);
//     EXPECT_TRUE(grid.isOutputEnabled()); // デフォルトtrue
//     EXPECT_FALSE(grid.hasOutputLabel());

//     grid.setOutputLabel("Vn");
//     EXPECT_TRUE(grid.hasOutputLabel());
//     EXPECT_EQ(grid.getOutputLabel(), "Vn");

//     grid.setOutputEnabled(false);
//     EXPECT_FALSE(grid.isOutputEnabled());
// }

// // Vn更新ロジックのシンプルテスト（1要素）
// TEST(Grid2DSEOTest, UpdateGridVnSimple) {
//     Grid2D<SEO> grid(1, 1);
//     auto seo = grid.getElement(0, 0);
//     seo->setQ(1.0);
//     seo->setUp(1, 1, 1, 1, 0, 0); // r, rj, cj, c, vd, legs

//     grid.updateGridVn();
//     // Q/Cj = 1/1 = 1.0 がベース（Cなどを考慮すると少し変動）
//     EXPECT_GT(seo->getVn(), 0.0);
// }

// // トンネル計算フローのテスト（強制的にdEを設定）
// TEST(Grid2DSEOTest, TunnelCalc) {
//     Grid2D<SEO> grid(1, 1);
//     auto seo = grid.getElement(0, 0);
//     seo->setUp(1, 1, 1, 1, 0, 0);
//     seo->setdE("up", 2.0);
//     seo->setdE("down", -1.0);

//     bool result = seo->calculateTunnelWt();
//     EXPECT_TRUE(result);
//     EXPECT_GT(seo->getWT()["up"], 0.0);
//     EXPECT_DOUBLE_EQ(seo->getWT()["down"], 0.0);
// }
