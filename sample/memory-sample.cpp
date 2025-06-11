//------------------------------------------------------------------------------------------
// memoryのサンプル
// 振動子一つとメモリ一つの接続
#include <iostream>
#include <fstream>
#include <memory>
#include <vector>
#include "seo_class.hpp"
#include "memory_class.hpp"
#include "grid_2dim.hpp"
#include "simulation_2d.hpp"
#include "constants.hpp"

constexpr double R = 0.5;
constexpr double Rj = 0.002;
constexpr double C = 2.0;
constexpr double Vd = -0.004;
constexpr double CL = 2.0;
constexpr double Vd_memory = 0.044;
constexpr double Cj_memory = 40;
constexpr double dt = 0.1;
constexpr double endtime = 200;
constexpr double Vth = 0.004;
double cj_leg1 = seo_junction_cj_calc(leg1, C, Vth);

using Grid = Grid2D<BaseElement>;
using Sim = Simulation2D<BaseElement>;

int main()
{
    // 振動子の初期化
    Grid grid_seo(1,1,false);
    // メモリの初期化（isMemoryをtrueで渡す）
    Grid grid_memory(1,1, false, true);

    // 初期化
    auto seo = std::make_shared<SEO>();
    seo->setUp(R, Rj, cj_leg1, C, Vd, leg1);
    grid_seo.setElement(0, 0, seo);

    auto memory = std::make_shared<MEMORY>();
    memory->setUp(Rj,Cj_memory,C,CL,Vd_memory,leg1);
    grid_memory.setElement(0, 0, memory);

    // 接続 
    seo->setConnections({memory});
    memory->setConnections({seo});

    // シミュレーション準備
    Sim sim(dt, endtime);
    sim.addGrid({grid_seo, grid_memory});

    // トリガ
    sim.addVoltageTrigger(100, &grid_seo, 0, 0, -0.004);

    // 出力ファイル設定
    auto ofs = std::make_shared<std::ofstream>("../output/memory.txt");
    sim.addSelectedElements(ofs, {seo, memory});
    sim.generateGnuplotScript("../output/memory.txt", {"seo", "memory"});

    // 実行
    sim.run();
    return 0;
}
