// 2層seoシミュレーション
// --------------------------------------------------------------------------------
#include <iostream>
#include <fstream>
#include <memory>
#include <vector>
#include "seo_class.hpp"
#include "grid_2dim.hpp"
#include "simulation_2d.hpp"
#include "oyl_video.hpp"
#include "constants.hpp"

constexpr int size_x = 32;
constexpr int size_y = 32;
constexpr double Vd = 0.004;
constexpr double R = 0.5;
constexpr double Rj = 0.002;
constexpr double Cj = Cj_leg5;
constexpr double C = 2.0;
constexpr double dt = 0.1;
constexpr double endtime = 200;

using Grid = Grid2D<BaseElement>;
using Sim = Simulation2D<BaseElement>;

int main()
{
    // === Gridを2層作成 ===
    Grid grid1(size_y, size_x, true);
    Grid grid2(size_y, size_x, true);

    grid1.setOutputLabel("seo1");
    grid2.setOutputLabel("seo2");

    // === 各GridにSEO素子を生成して配置 ===
    for (int y = 0; y < size_y; ++y)
    {
        for (int x = 0; x < size_x; ++x)
        {
            {
                auto seo = std::make_shared<SEO>();
                double biasVd = ((x + y) % 2 == 0) ? Vd : -Vd;
                seo->setUp(R, Rj, Cj, C, biasVd, 5);
                grid1.setElement(y, x, seo);
            }
            {
                auto seo = std::make_shared<SEO>();
                double biasVd = ((x * y) % 2 == 0) ? -Vd : Vd; // 少し違う初期化
                seo->setUp(R, Rj, Cj, C, biasVd, 5);
                grid2.setElement(y, x, seo);
            }
        }
    }

    // === 接続設定（上下左右＋上下層） ===
    for (int y = 0; y < size_y; ++y)
    {
        for (int x = 0; x < size_x; ++x)
        {
            {
                auto center = grid1.getElement(y, x);
                std::vector<std::shared_ptr<BaseElement>> neighbors;
                if (y > 0) neighbors.push_back(grid1.getElement(y - 1, x));
                if (x < size_x - 1) neighbors.push_back(grid1.getElement(y, x + 1));
                if (y < size_y - 1) neighbors.push_back(grid1.getElement(y + 1, x));
                if (x > 0) neighbors.push_back(grid1.getElement(y, x - 1));
                neighbors.push_back(grid2.getElement(y, x)); // grid2との接続
                center->setConnections(neighbors);
            }

            {
                auto center = grid2.getElement(y, x);
                std::vector<std::shared_ptr<BaseElement>> neighbors;
                if (y > 0) neighbors.push_back(grid2.getElement(y - 1, x));
                if (x < size_x - 1) neighbors.push_back(grid2.getElement(y, x + 1));
                if (y < size_y - 1) neighbors.push_back(grid2.getElement(y + 1, x));
                if (x > 0) neighbors.push_back(grid2.getElement(y, x - 1));
                neighbors.push_back(grid1.getElement(y, x)); // grid1との接続
                center->setConnections(neighbors);
            }
        }
    }

    // === シミュレーション初期化 ===
    Sim sim(dt, endtime);
    sim.addGrid({grid1, grid2});

    // === 特定素子の出力設定 ===
    auto ofs = std::make_shared<std::ofstream>("../output/multivn.txt");
    std::vector<std::shared_ptr<BaseElement>> targets = {
        grid1.getElement(15, 15),
        grid2.getElement(15, 15),
        grid1.getElement(16, 15),
        grid2.getElement(16, 15)
    };
    sim.addSelectedElements(ofs, targets);
    std::vector<std::string> labels = {"g1_1515", "g2_1515", "g1_1615", "g2_1615"};
    sim.generateGnuplotScript("../output/multivn.txt", labels);

    // === トリガ設定 ===
    sim.addVoltageTrigger(100, &grid1, 15, 15, 0.006);

    // === 実行 ===
    sim.run();

    // === 動画出力 ===
    const auto& outputs = sim.getOutputs();
    for (const auto& [label, data] : outputs)
    {
        auto normalized = oyl::normalizeto255(data);
        oyl::VideoClass video(normalized);
        video.set_filename("../output/" + label + ".mp4");
        video.set_codec(cv::VideoWriter::fourcc('m', 'p', '4', 'v'));
        video.set_fps(30.0);
        video.makevideo();
    }

    return 0;
}
