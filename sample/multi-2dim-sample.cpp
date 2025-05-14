// --------------------------------------------------------------------------------
// multi-seoの2次元シミュレーション
#include <iostream>
#include "multi_seo_class.hpp"
#include "grid_2dim.hpp"
#include "simulation_2d.hpp"
#include "oyl_video.hpp"

// 各種定数定義（シミュレーションパラメータ）
constexpr int size_x = 32;
constexpr int size_y = 32;
constexpr double Vd = 0.004;               // バイアス電圧
constexpr double R = 1;                   // 抵抗
constexpr double Rj = 0.001;              // トンネル抵抗
constexpr double Cj = multi_Cj;           // 接合容量（定数ファイルから）
constexpr double C = 2.0;                 // 接続容量
constexpr double dt = 0.1;                // 時間刻み幅
constexpr double endtime = 300;           // シミュレーション終了時間
constexpr int multi_num = 20;             // 多重ジャンクション数

// テンプレートでベース素子型を統一
using Grid = Grid2D<BaseElement>;
using Sim = Simulation2D<BaseElement>;

int main()
{
    Grid grid(size_y, size_x, true);
    grid.setOutputLabel("multiseo"); // 出力識別用ラベル

    // 各セルにMultiSEO素子を生成・初期化・配置
    for (int y = 0; y < size_y; ++y) {
        for (int x = 0; x < size_x; ++x) {
            auto multiseo = std::make_shared<MultiSEO>();
            double biasVd = ((x + y) % 2 == 0) ? Vd : -Vd;
            grid.setElement(y, x, multiseo);
            multiseo->setUp(R, Rj, Cj, C, biasVd, 4, multi_num); // 仮に legs = 4 としておく
        }
    }

    // 全体に対して接続設定（要素生成後に実行）
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

    Sim sim(dt, endtime);
    sim.addGrid({grid});

    auto ofs = std::make_shared<std::ofstream>("../output/multivn.txt");
    std::vector<std::shared_ptr<BaseElement>> targets = {
        grid.getElement(15, 15),
        grid.getElement(16, 15),
        grid.getElement(17, 15)
    };
    sim.addSelectedElements(ofs, targets);
    std::vector<std::string> labels = {"1515", "1615", "1715"};
    sim.generateGnuplotScript("../output/multivn.txt", labels);

    sim.addVoltageTrigger(150, &grid, 15, 15, 0.006);
    sim.run();

    const auto& outputs = sim.getOutputs();
    const auto& data = outputs.count("multiseo") ? outputs.at("multiseo") : std::vector<std::vector<std::vector<double>>>();

    if (!data.empty())
    {
        auto normalized = oyl::normalizeto255(data);
        oyl::VideoClass video(normalized);
        video.set_filename("../output/multiseo.mp4");
        video.set_codec(cv::VideoWriter::fourcc('m', 'p', '4', 'v'));
        video.set_fps(30.0);
        video.makevideo();
    }
    else
    {
        std::cerr << "[ERROR] No output data found for label 'multiseo'" << std::endl;
    }

    return 0;
}