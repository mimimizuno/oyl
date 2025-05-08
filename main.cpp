// seoの２次元シミュレーション
// --------------------------------------------------------------------------------
#include <iostream>
#include <fstream>
#include <memory>
#include <vector>
#include "seo_class.hpp"
#include "grid_2dim.hpp"
#include "simulation_2d.hpp"
#include "oyl_video.hpp"

constexpr int size_x = 32;
constexpr int size_y = 32;
constexpr double Vd = 0.0044;
constexpr double R = 0.5;
constexpr double Rj = 0.002;
constexpr double Cj = 10.0;
constexpr double C = 2.0;
constexpr double dt = 0.1;
constexpr double endtime = 200;

using Grid = Grid2D<BaseElement>;
using Sim = Simulation2D<BaseElement>;

int main()
{
    Grid grid(size_y, size_x, true);
    grid.setOutputLabel("seo");

    // SEO素子の生成と配置
    for (int y = 0; y < size_y; ++y)
    {
        for (int x = 0; x < size_x; ++x)
        {
            auto seo = std::make_shared<SEO>();
            double biasVd = ((x + y) % 2 == 0) ? Vd : -Vd;
            seo->setUp(R, Rj, Cj, C, biasVd, 4);
            grid.setElement(y, x, seo);
        }
    }

    // 接続設定（上下左右）
    for (int y = 0; y < size_y; ++y)
    {
        for (int x = 0; x < size_x; ++x)
        {
            auto center = grid.getElement(y, x);
            std::vector<std::shared_ptr<BaseElement>> neighbors;
            if (y > 0) neighbors.push_back(grid.getElement(y - 1, x));
            if (x < size_x - 1) neighbors.push_back(grid.getElement(y, x + 1));
            if (y < size_y - 1) neighbors.push_back(grid.getElement(y + 1, x));
            if (x > 0) neighbors.push_back(grid.getElement(y, x - 1));
            center->setConnections(neighbors);
        }
    }

    // シミュレーション初期化
    Sim sim(dt, endtime);
    sim.addGrid({grid});

    // 特定素子の出力設定
    auto ofs = std::make_shared<std::ofstream>("../output/multivn.txt");
    std::vector<std::shared_ptr<BaseElement>> targets = {
        grid.getElement(15, 15),
        grid.getElement(16, 15),
        grid.getElement(17, 15)
    };
    sim.addSelectedElements(ofs, targets);
    std::vector<std::string> labels = {"1515", "1615", "1715"};
    sim.generateGnuplotScript("../output/multivn.txt", labels);

    // トリガ設定
    sim.addVoltageTrigger(100, &grid, 15, 15, 0.006);
    sim.run();

    // 動画出力
    const auto& outputs = sim.getOutputs();
    if (outputs.count("seo"))
    {
        const auto& data = outputs.at("seo");
        auto normalized = oyl::normalizeto255(data);
        oyl::VideoClass video(normalized);
        video.set_filename("../output/seo.mp4");
        video.set_codec(cv::VideoWriter::fourcc('m', 'p', '4', 'v'));
        video.set_fps(30.0);
        video.makevideo();
    }
    else
    {
        std::cerr << "[ERROR] No output data found for label 'seo'" << std::endl;
    }

    return 0;
}

// --------------------------------------------------------------------------------

// multi-seoの2次元シミュレーション
// #include <iostream>
// #include "multi_seo_class.hpp"
// #include "grid_2dim.hpp"
// #include "simulation_2d.hpp"
// #include "oyl_video.hpp"

// constexpr int size_x = 32;
// constexpr int size_y = 32;
// constexpr double Vd = 0.004;
// constexpr double R = 1;
// constexpr double Rj = 0.001;
// constexpr double Cj = multi_Cj;
// constexpr double C = 2.0;
// constexpr double dt = 0.1;
// constexpr double endtime = 300;
// constexpr int multi_num = 20;

// using Grid = Grid2D<MultiSEO>;
// using Sim = Simulation2D<MultiSEO>;

// int main()
// {
//     Grid grid(size_y, size_x, true);
//     grid.setOutputLabel("multiseo");

//     // // 多重振動子の初期化と接続
//     // for (int y = 0; y < size_y; ++y)
//     // {
//     //     for (int x = 0; x < size_x; ++x)
//     //     {
//     //         auto multiseo = grid.getElement(y, x);
//     //         double biasVd = ((x + y) % 2 == 0) ? Vd : -Vd;
//     //         multiseo->setUp(R, Rj, Cj, C, biasVd, 4, multi_num);

//     //         std::vector<std::shared_ptr<MultiSEO>> connections;
//     //         if (y > 0) connections.push_back(grid.getElement(y - 1, x));         // 上
//     //         if (x < size_x - 1) connections.push_back(grid.getElement(y, x + 1));// 右
//     //         if (y < size_y - 1) connections.push_back(grid.getElement(y + 1, x));// 下
//     //         if (x > 0) connections.push_back(grid.getElement(y, x - 1));         // 左

//     //         multiseo->setConnections(connections);
//     //     }
//     // }

//     for (int y = 0; y < size_y; ++y) {
//         for (int x = 0; x < size_x; ++x) {
//             auto multiseo = grid.getElement(y, x);
    
//             // 外枠ならVd=0、legs=0
//             if (x == 0 || x == size_x - 1 || y == 0 || y == size_y - 1) {
//                 multiseo->setUp(R, Rj, Cj, C, 0.0, 0, multi_num);
//                 continue;
//             }
    
//             // 中心部
//             double biasVd = ((x + y) % 2 == 0) ? Vd : -Vd;
    
//             // 接続候補
//             std::vector<std::shared_ptr<MultiSEO>> connections;
//             if (y > 0) connections.push_back(grid.getElement(y - 1, x));
//             if (x < size_x - 1) connections.push_back(grid.getElement(y, x + 1));
//             if (y < size_y - 1) connections.push_back(grid.getElement(y + 1, x));
//             if (x > 0) connections.push_back(grid.getElement(y, x - 1));
    
//             int legs = connections.size(); // 実際の接続数に応じて設定
//             multiseo->setUp(R, Rj, Cj, C, biasVd, legs, multi_num);
//             multiseo->setConnections(connections);
//         }
//     }
    

//     Sim sim(dt, endtime);
//     sim.addGrid({grid});

//     // トリガ設定：時刻150ns〜151nsの間、(15,15)の素子に0.006Vを加える
//     sim.addVoltageTrigger(150, &grid, 15, 15, 0.006);
//     std::ofstream selectedFile1("../output/trrigerseo1515.txt");
//     auto elem1 = grid.getElement(15,15);
//     sim.addSelectedElement(selectedFile1, elem1);
//     std::ofstream selectedFile2("../output/trrigerseo11.txt");
//     auto elem2 = grid.getElement(1,1);
//     sim.addSelectedElement(selectedFile2, elem2);
//     std::ofstream selectedFile3("../output/trrigerseo3030.txt");
//     auto elem3 = grid.getElement(30,30);
//     sim.addSelectedElement(selectedFile3, elem3);
//     // 実行
//     sim.run();

//     // 出力処理
//     const auto& outputs = sim.getOutputs();
//     if (outputs.count("multiseo"))
//     {
//         const auto& data = outputs.at("multiseo");
//         auto normalized = oyl::normalizeto255(data);
//         std::string filepath = "../output/multiseo.mp4";

//         oyl::VideoClass video(normalized);
//         video.set_filename(filepath);
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
// #include <iostream>
// #include <fstream>
// #include <memory>
// #include <vector>
// #include "seo_class.hpp"
// #include "oneway_unit.hpp"
// #include "grid_2dim.hpp"
// #include "simulation_2d.hpp"
// #include "constants.hpp"

// constexpr double R = 0.5;
// constexpr double Rj = 0.002;
// constexpr double C = 2.0;
// constexpr double Vd = 0.0044;
// constexpr double dt = 0.1;
// constexpr double endtime = 100;

// using Unit = OnewayUnit<SEO>;
// using Grid = Grid2D<Unit>;
// using Sim = Simulation2D<Unit>;

// int main()
// {
//     // 1x1のGridを作成
//     Grid grid(3, 3, true); // 境界対策として3x3（中央の1マスのみ使用）
//     grid.setOutputLabel("oneway");

//     // 中央に一つだけOnewayUnitを配置
//     auto unit = grid.getElement(1, 1);
//     unit->setOnewayDirection("right");
//     unit->setOnewaySeoParam(R, Rj, Cj_leg2, Cj_leg3, C, Vd);

//     // SEOとの接続設定（dummyでもよい）
//     std::shared_ptr<SEO> dummy_left = std::make_shared<SEO>();
//     std::shared_ptr<SEO> dummy_right = std::make_shared<SEO>();
//     unit->setOnewayConnections(dummy_left, dummy_right);

//     // シミュレーション準備
//     Sim sim(dt, endtime);
//     sim.addGrid({grid});

//     // トリガ：100ns〜101nsの間、内部素子0番に-0.006V加える
//     auto internals = unit->getInternalElements();
//     internals[0]->setVsum(internals[0]->getSurroundingVsum() - 0.006);

//     // 電位変化を出力
//     auto ofs = std::make_shared<std::ofstream>("../output/oneway_vn.txt");
//     std::vector<std::shared_ptr<SEO>> targets;
//     auto internals = unit->getInternalElements();
//     for (const auto& elem : internals) {
//         if (elem) targets.push_back(elem);
//     }
//     sim.addSelectedElements(ofs, targets); // Simulation2D<SEO> のときにOK


//     std::vector<std::string> labels = {"ow0", "ow1", "ow2", "ow3"};
//     sim.generateGnuplotScript("../output/oneway_vn.txt", labels);

//     // 実行
//     sim.run();

//     std::cout << "Simulation finished. Output saved to oneway_vn.txt" << std::endl;
//     return 0;
// }
