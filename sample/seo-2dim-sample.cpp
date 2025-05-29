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
            seo->setUp(R, Rj, Cj, C, biasVd, leg4);
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