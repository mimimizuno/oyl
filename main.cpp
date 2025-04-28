#include <iostream>
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

using Sim = Simulation2D<SEO>;
using Grid = Grid2D<SEO>;

int main()
{
    Grid grid(size_y, size_x, true);
    grid.setOutputLabel("seo");


    // SEO初期化と接続
    for (int y = 0; y < size_y; ++y)
    {
        for (int x = 0; x < size_x; ++x)
        {
            auto seo = grid.getElement(y, x);
            double biasVd = ((x + y) % 2 == 0) ? Vd : -Vd;
            seo->setUp(R, Rj, Cj, C, biasVd, 4);
            std::vector<std::shared_ptr<SEO>> connections;
            if (y > 0) connections.push_back(grid.getElement(y - 1, x));     // 上
            if (x < size_x - 1) connections.push_back(grid.getElement(y, x + 1)); // 右
            if (y < size_y - 1) connections.push_back(grid.getElement(y + 1, x)); // 下
            if (x > 0) connections.push_back(grid.getElement(y, x - 1));     // 左
            seo->setConnections(connections);
        }
    }
    Sim sim(dt, endtime);
    sim.addGrid({grid});
    std::ofstream selectedFile1("../output/trrigerseo1515.txt");
    auto elem1 = grid.getElement(15,15);
    sim.addSelectedElement(selectedFile1, elem1);
    std::ofstream selectedFile2("../output/trrigerseo1615.txt");
    auto elem2 = grid.getElement(16,15);
    sim.addSelectedElement(selectedFile2, elem2);
    std::ofstream selectedFile3("../output/trrigerseo1715.txt");
    auto elem3 = grid.getElement(17,15);
    sim.addSelectedElement(selectedFile3, elem3);
    // 時刻150ns〜150.1nsの間、(1,1)の素子に0.006Vを加える
    sim.addVoltageTrigger(100, &grid, 15, 15, 0.006);
    sim.run();


    // 出力処理
    const auto& outputs = sim.getOutputs();
    if (outputs.count("seo"))
    {
        const auto& data = outputs.at("seo");
        auto normalized = oyl::normalizeto255(data);
        std::string label = grid.getOutputLabel();
        std::string filepath = "../output/" + label + ".mp4";

        oyl::VideoClass video(normalized);
        video.set_filename(filepath);
        video.set_codec(cv::VideoWriter::fourcc('m', 'p', '4', 'v')); // mp4対応コーデック
        video.set_fps(30.0);
        // video.set_scaleBar(1).set_cellsize(20);
        video.makevideo();
    }
    else
    {
        std::cerr << "[ERROR] No output data found for label 'seo'" << std::endl;
    }

    return 0;
}
