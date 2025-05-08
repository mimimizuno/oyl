// seoの２次元シミュレーション
// --------------------------------------------------------------------------------
// #include <iostream>
// #include <fstream>
// #include <memory>
// #include <vector>
// #include "seo_class.hpp"
// #include "grid_2dim.hpp"
// #include "simulation_2d.hpp"
// #include "oyl_video.hpp"

// constexpr int size_x = 32;
// constexpr int size_y = 32;
// constexpr double Vd = 0.0044;
// constexpr double R = 0.5;
// constexpr double Rj = 0.002;
// constexpr double Cj = 10.0;
// constexpr double C = 2.0;
// constexpr double dt = 0.1;
// constexpr double endtime = 200;

// using Grid = Grid2D<BaseElement>;
// using Sim = Simulation2D<BaseElement>;

// int main()
// {
//     Grid grid(size_y, size_x, true);
//     grid.setOutputLabel("seo");

//     // SEO素子の生成と配置
//     for (int y = 0; y < size_y; ++y)
//     {
//         for (int x = 0; x < size_x; ++x)
//         {
//             auto seo = std::make_shared<SEO>();
//             double biasVd = ((x + y) % 2 == 0) ? Vd : -Vd;
//             seo->setUp(R, Rj, Cj, C, biasVd, 4);
//             grid.setElement(y, x, seo);
//         }
//     }

//     // 接続設定（上下左右）
//     for (int y = 0; y < size_y; ++y)
//     {
//         for (int x = 0; x < size_x; ++x)
//         {
//             auto center = grid.getElement(y, x);
//             std::vector<std::shared_ptr<BaseElement>> neighbors;
//             if (y > 0) neighbors.push_back(grid.getElement(y - 1, x));
//             if (x < size_x - 1) neighbors.push_back(grid.getElement(y, x + 1));
//             if (y < size_y - 1) neighbors.push_back(grid.getElement(y + 1, x));
//             if (x > 0) neighbors.push_back(grid.getElement(y, x - 1));
//             center->setConnections(neighbors);
//         }
//     }

//     // シミュレーション初期化
//     Sim sim(dt, endtime);
//     sim.addGrid({grid});

//     // 特定素子の出力設定
//     auto ofs = std::make_shared<std::ofstream>("../output/multivn.txt");
//     std::vector<std::shared_ptr<BaseElement>> targets = {
//         grid.getElement(15, 15),
//         grid.getElement(16, 15),
//         grid.getElement(17, 15)
//     };
//     sim.addSelectedElements(ofs, targets);
//     std::vector<std::string> labels = {"1515", "1615", "1715"};
//     sim.generateGnuplotScript("../output/multivn.txt", labels);

//     // トリガ設定
//     sim.addVoltageTrigger(100, &grid, 15, 15, 0.006);
//     sim.run();

//     // 動画出力
//     const auto& outputs = sim.getOutputs();
//     if (outputs.count("seo"))
//     {
//         const auto& data = outputs.at("seo");
//         auto normalized = oyl::normalizeto255(data);
//         oyl::VideoClass video(normalized);
//         video.set_filename("../output/seo.mp4");
//         video.set_codec(cv::VideoWriter::fourcc('m', 'p', '4', 'v'));
//         video.set_fps(30.0);
//         video.makevideo();
//     }
//     else
//     {
//         std::cerr << "[ERROR] No output data found for label 'seo'" << std::endl;
//     }

//     return 0;
// }

// --------------------------------------------------------------------------------
// multi-seoの2次元シミュレーション
// #include <iostream>
// #include "multi_seo_class.hpp"
// #include "grid_2dim.hpp"
// #include "simulation_2d.hpp"
// #include "oyl_video.hpp"

// // 各種定数定義（シミュレーションパラメータ）
// constexpr int size_x = 32;
// constexpr int size_y = 32;
// constexpr double Vd = 0.004;               // バイアス電圧
// constexpr double R = 1;                   // 抵抗
// constexpr double Rj = 0.001;              // トンネル抵抗
// constexpr double Cj = multi_Cj;           // 接合容量（定数ファイルから）
// constexpr double C = 2.0;                 // 接続容量
// constexpr double dt = 0.1;                // 時間刻み幅
// constexpr double endtime = 300;           // シミュレーション終了時間
// constexpr int multi_num = 20;             // 多重ジャンクション数

// // テンプレートでベース素子型を統一
// using Grid = Grid2D<BaseElement>;
// using Sim = Simulation2D<BaseElement>;

// int main()
// {
//     Grid grid(size_y, size_x, true);
//     grid.setOutputLabel("multiseo"); // 出力識別用ラベル

//     // 各セルにMultiSEO素子を生成・初期化・配置
//     for (int y = 0; y < size_y; ++y) {
//         for (int x = 0; x < size_x; ++x) {
//             auto multiseo = std::make_shared<MultiSEO>();
//             double biasVd = ((x + y) % 2 == 0) ? Vd : -Vd;
//             grid.setElement(y, x, multiseo);
//             multiseo->setUp(R, Rj, Cj, C, biasVd, 4, multi_num); // 仮に legs = 4 としておく
//         }
//     }

//     // 全体に対して接続設定（要素生成後に実行）
//     for (int y = 0; y < size_y; ++y)
//     {
//         for (int x = 0; x < size_x; ++x)
//         {
//             auto center = grid.getElement(y, x);
//             std::vector<std::shared_ptr<BaseElement>> neighbors;
//             if (y > 0) neighbors.push_back(grid.getElement(y - 1, x));
//             if (x < size_x - 1) neighbors.push_back(grid.getElement(y, x + 1));
//             if (y < size_y - 1) neighbors.push_back(grid.getElement(y + 1, x));
//             if (x > 0) neighbors.push_back(grid.getElement(y, x - 1));
//             center->setConnections(neighbors);
//         }
//     }

//     Sim sim(dt, endtime);
//     sim.addGrid({grid});

//     auto ofs = std::make_shared<std::ofstream>("../output/multivn.txt");
//     std::vector<std::shared_ptr<BaseElement>> targets = {
//         grid.getElement(15, 15),
//         grid.getElement(16, 15),
//         grid.getElement(17, 15)
//     };
//     sim.addSelectedElements(ofs, targets);
//     std::vector<std::string> labels = {"1515", "1615", "1715"};
//     sim.generateGnuplotScript("../output/multivn.txt", labels);

//     sim.addVoltageTrigger(150, &grid, 15, 15, 0.006);
//     sim.run();

//     const auto& outputs = sim.getOutputs();
//     const auto& data = outputs.count("multiseo") ? outputs.at("multiseo") : std::vector<std::vector<std::vector<double>>>();

//     if (!data.empty())
//     {
//         auto normalized = oyl::normalizeto255(data);
//         oyl::VideoClass video(normalized);
//         video.set_filename("../output/multiseo.mp4");
//         video.set_codec(cv::VideoWriter::fourcc('m', 'p', '4', 'v'));
//         video.set_fps(30.0);
//         video.makevideo();
//     }
//     else
//     {
//         std::cerr << "[ERROR] No output data found for label 'multiseo'" << std::endl;
//     }

//     return 0;
// }
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
