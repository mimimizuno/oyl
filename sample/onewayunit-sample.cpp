//------------------------------------------------------------------------------------------
// onewayのテスト
// oneway unit を使ったテストシミュレーション
#include <iostream>
#include <fstream>
#include <memory>
#include <vector>
#include "seo_class.hpp"
#include "oneway_unit.hpp"
#include "grid_2dim.hpp"
#include "simulation_2d.hpp"
#include "constants.hpp"

constexpr double R = 0.5;
constexpr double Rj = 0.002;
constexpr double C = 2.0;
constexpr double Vd = 0.0044;
constexpr double dt = 0.1;
constexpr double endtime = 100;

using Grid = Grid2D<BaseElement>;
using Sim = Simulation2D<BaseElement>;

int main()
{
    Grid grid(3, 3, true); // 境界対策で3x3、中央を使用
    grid.setOutputLabel("oneway");

    // 内部SEOを初期化して設定
    std::array<std::shared_ptr<BaseElement>, 4> internal_seos;
    for (int i = 0; i < 4; ++i)
    {
        internal_seos[i] = std::make_shared<SEO>();
    }

    // OnewayUnitを生成して中央に配置
    auto unit = std::make_shared<OnewayUnit>("right");
    unit->setInternalElements(internal_seos);
    unit->setOnewaySeoParam(R, Rj, Cj_leg2, Cj_leg3, C, Vd);

    // SEOの内部素子取得（再取得）
    const auto& internals = unit->getInternalElements();

    // dummy接続SEOを生成して grid に登録
    auto dummy_left = std::make_shared<SEO>();
    dummy_left->setUp(R, Rj, C, C, 0.0, 1);
    grid.setElement(1, 0, dummy_left); // ← gridに配置

    auto dummy_right = std::make_shared<SEO>();
    dummy_right->setUp(R, Rj, C, C, 0.0, 1);
    grid.setElement(1, 2, dummy_right); // ← gridに配置

    unit->setOnewayConnections(dummy_left, dummy_right);

    // グリッドにユニットを配置
    grid.setElement(1, 1, unit);

    // 未使用セルを全て安全なダミーで初期化
    for (int y = 0; y < grid.numRows(); ++y) {
        for (int x = 0; x < grid.numCols(); ++x) {
            if (!grid.getElement(y, x)) {
                grid.setElement(y, x, std::make_shared<SEO>());
            }
        }
    }

    // シミュレーション準備
    Sim sim(dt, endtime);
    sim.addGrid({grid});

    // トリガ：素子0に -0.006V を印加
    internals[0]->setVsum(internals[0]->getSurroundingVsum() - 0.006);

    // トリガ：t = 100 で dummy_left に +0.006V を印加
    sim.addVoltageTrigger(100, &grid, 1, 0, 0.006); // dummy_left は (1,0)

    // 出力ファイル設定
    auto ofs = std::make_shared<std::ofstream>("../output/oneway_vn.txt");
    sim.addSelectedElements(ofs, {dummy_left, internals[0], internals[1], internals[2], internals[3], dummy_right});
    sim.generateGnuplotScript("../output/oneway_vn.txt", {"dummyL", "ow0", "ow1", "ow2", "ow3", "dummyR"});

    // 実行
    sim.run();
    std::cout << "Simulation finished. Output saved to oneway_vn.txt" << std::endl;
    return 0;
}
