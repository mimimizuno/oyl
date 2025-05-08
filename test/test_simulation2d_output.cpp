// #include "gtest/gtest.h"
// #include "simulation_2d.hpp"
// #include "seo_class.hpp"
// #include "grid_2dim.hpp"

// using Sim = Simulation2D<SEO>;

// TEST(Simulation2DTest, ConstructorInitializesCorrectly) {
//     Sim sim(0.01, 1.0);
//     EXPECT_TRUE(sim.getGrids().empty());
// }

// TEST(Simulation2DTest, AddGridStoresGridCorrectly) {
//     Sim sim(0.01, 1.0);
//     Grid2D<SEO> grid(2, 2);
//     sim.addGrid({grid});
//     EXPECT_EQ(sim.getGrids().size(), 1);
//     EXPECT_EQ(sim.getGrids()[0].numRows(), 2);
// }

// TEST(Simulation2DTest, GetOutputsInitiallyEmpty) {
//     Sim sim(0.01, 1.0);
//     EXPECT_TRUE(sim.getOutputs().empty());
// }

// TEST(Simulation2DTest, OutputToOylStoresValuesWhenTimeExceeds) {
//     Sim sim(0.01, 1.0);
//     Grid2D<SEO> grid(2, 2);
//     grid.setOutputLabel("grid1");
//     grid.setOutputEnabled(true);

//     grid.getElement(0, 0)->setVn(0.5);
//     sim.addGrid({grid});

//     for (int i = 0; i < 20; ++i) sim.runStep(); // t >= 0.1 にする

//     auto out = sim.getOutputs();
//     ASSERT_TRUE(out.find("grid1") != out.end());
// }

// TEST(Simulation2DTest, CompareWtReturnsFalseIfNoTunnel) {
//     Sim sim(0.01, 1.0);
//     Grid2D<SEO> grid(1, 1);
//     sim.addGrid({grid});
//     auto result = sim.comparewt();
//     EXPECT_FALSE(result.first);
//     EXPECT_EQ(result.second, nullptr);
// }

// TEST(Simulation2DTest, HandleTunnelsModifiesCharge) {
//     Sim sim(0.01, 1.0);
//     Grid2D<SEO> grid(1, 1);
//     auto elem = grid.getElement(0, 0);
//     elem->setdE("up", 0.2);
//     elem->calculateTunnelWt();
//     grid.gridminwt(1.0); // tunnelplaceを更新
//     sim.handleTunnels(grid);
//     EXPECT_NEAR(elem->getQ(), -0.1602, 1e-4); // e = 0.1602
// }

// TEST(Simulation2DTest, RunStepIncrementsTime) {
//     Sim sim(0.01, 1.0);
//     Grid2D<SEO> grid(3, 3);
//     sim.addGrid({grid});
//     sim.runStep(); // t += dt
//     for (int i = 0; i < 19; ++i) sim.runStep();
//     EXPECT_GE(sim.getOutputs().size(), 1);
// }

// TEST(Simulation2DTest, RunTerminatesAtEndtime) {
//     Sim sim(0.01, 0.3); // 時間を伸ばすと出力が増える
//     Grid2D<SEO> grid(3, 3);
//     grid.setOutputLabel("test");
//     sim.addGrid({grid});
//     sim.run(); // 完走する

//     const auto& outputs = sim.getOutputs();
//     ASSERT_TRUE(outputs.find("test") != outputs.end()); // ラベルがあること
//     EXPECT_GE(outputs.at("test").size(), 2); // 出力時刻2回以上
// }
