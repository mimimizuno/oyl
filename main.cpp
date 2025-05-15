// seo particle computation test
// --------------------------------------------------------------------------------
#include <iostream>
#include <fstream>
#include <memory>
#include <vector>
#include "seo_class.hpp"
#include "oneway_unit.hpp"
#include "grid_2dim.hpp"
#include "simulation_2d.hpp"
#include "oyl_video.hpp"
#include "constants.hpp"
#include "particle_computation_methods.hpp"

constexpr int particles = 2;
constexpr int size_x = 17;
constexpr int size_y = 11;
constexpr double Vd_seo = 0.004;
constexpr double Vd_oneway = 0.0039;
constexpr double R = 1.5;
constexpr double R_small = 0.8;
constexpr double Rj = 0.001;
constexpr double C = 2.0;
constexpr double dt = 0.1;
constexpr double endtime = 200;

using Grid = Grid2D<BaseElement>;
using Sim = Simulation2D<BaseElement>;
// onewayはサイズ統一しちゃった方が楽そう。command_downとか動画化するものだけちゃんとサイズを整える。
int main()
{
    std::vector<std::vector<int>> maze = {
        {0,0,0,0,0,0,0,1,0,1,0,1,0,1,0},
        {0,0,1,0,0,0,0,1,0,1,0,1,0,1,0},
        {0,1,1,1,1,0,0,1,0,1,0,1,0,1,0},
        {0,1,1,1,1,1,1,1,1,1,1,1,1,1,0},
        {0,0,1,0,1,0,0,0,0,0,0,1,0,1,0},
        {0,0,1,0,1,0,0,1,0,0,0,1,0,1,0},
        {0,0,1,0,1,0,1,1,1,1,0,1,0,1,0},
        {0,0,1,0,1,0,1,1,1,1,1,1,1,1,0},
        {0,0,1,0,1,0,0,1,0,1,0,0,0,0,0},
    };
    // === Gridを生成 ===
    Grid command_down(size_y, size_x * particles - 2);                    // 命令方向回路（下）
    Grid detection_down(size_y, size_x);                              // 衝突方向回路（下）
    Grid command_left(size_y * particles - 2, size_x);                    // 命令方向回路（左）
    Grid detection_left(size_y, size_x);                              // 衝突判定回路（左）
    Grid command_up(size_y, size_x * particles - 2);                      // 命令方向回路（上）
    Grid detection_up(size_y, size_x);                                // 衝突方向回路（上）
    Grid command_right(size_y * particles - 2, size_x);                   // 命令方向回路（右）
    Grid detection_right(size_y, size_x);                             // 衝突判定回路（右）
    Grid oneway_command_down(size_y * particles, size_x * particles, false);  // 命令方向回路（下）における一方通行回路
    Grid oneway_CtoD_down(size_y * particles, size_x * particles, false);     // 命令方向回路（下）から衝突判定回路（下）をつなぐ一方通行回路
    Grid oneway_DtoC_downtoleft(size_y * particles, size_x * particles, false);   // 衝突判定回路（下）から命令方向回路（左）をつなぐ一方通行回路
    Grid oneway_command_left(size_y * particles, size_x * particles, false);  // 命令方向回路（左）における一方通行回路
    Grid oneway_CtoD_left(size_y * particles, size_x * particles, false);     // 命令方向回路（左）から衝突判定回路（左）をつなぐ一方通行回路
    Grid oneway_DtoC_lefttoup(size_y * particles, size_x * particles, false);     // 衝突判定回路（左）から命令方向回路（上）をつなぐ一方通行回路
    Grid oneway_command_up(size_y * particles, size_x * particles, false);    // 命令方向回路（下）における一方通行回路
    Grid oneway_CtoD_up(size_y * particles, size_x * particles, false);       // 命令方向回路（下）から衝突判定回路（下）をつなぐ一方通行回路
    Grid oneway_DtoC_uptoright(size_y * particles, size_x * particles, false);    // 衝突判定回路（下）から命令方向回路（左）をつなぐ一方通行回路
    Grid oneway_command_right(size_y * particles, size_x * particles, false); // 命令方向回路（左）における一方通行回路
    Grid oneway_CtoD_right(size_y * particles, size_x * particles, false);    // 命令方向回路（左）から衝突判定回路（左）をつなぐ一方通行回路
    Grid oneway_DtoC_righttodown(size_y * particles, size_x * particles, false);  // 衝突判定回路（左）から命令方向回路（上）をつなぐ一方通行回路

    // 動画出力するgridに名前をつける
    command_down.setOutputLabel("command_down");
    detection_down.setOutputLabel("detection_down");
    command_left.setOutputLabel("command_left");
    detection_left.setOutputLabel("detection_left");
    command_up.setOutputLabel("command_up");
    detection_up.setOutputLabel("detection_up");
    command_right.setOutputLabel("command_right");
    detection_right.setOutputLabel("detection_right");

    // === 全グリッド初期化 ===
    // 命令方向回路 (command_down, command_up)
    for (int y = 0; y < command_down.numRows(); ++y) {
        for (int x = 0; x < command_down.numCols(); ++x) {
            {
                auto seo = std::make_shared<SEO>();
                seo->setUp(R, Rj, Cj_leg6, C, Vd_seo, 6);
                command_down.setElement(y, x, seo);
            }
            {
                auto seo = std::make_shared<SEO>();
                seo->setUp(R, Rj, Cj_leg6, C, Vd_seo, 6);
                command_up.setElement(y, x, seo);
            }
        }
    }
    // 命令方向回路(command_left, command_right)
    for (int y = 0; y < command_left.numRows(); ++y) {
        for (int x = 0; x < command_left.numCols(); ++x) {
            {
                auto seo = std::make_shared<SEO>();
                seo->setUp(R, Rj, Cj_leg6, C, Vd_seo, 6);
                command_left.setElement(y, x, seo);
            }
            {
                auto seo = std::make_shared<SEO>();
                seo->setUp(R, Rj, Cj_leg6, C, Vd_seo, 6);
                command_right.setElement(y, x, seo);
            }
        }
    }

    // 衝突判定回路 (detection_down, detection_up, detection_left, detection_right)
    for (int y = 0; y < detection_down.numRows(); ++y) {
        for (int x = 0; x < detection_down.numCols(); ++x) {
            {
                auto seo = std::make_shared<SEO>();
                seo->setUp(R, Rj, Cj_leg4, C, Vd_seo, 4);
                detection_down.setElement(y, x, seo);
            }
            {
                auto seo = std::make_shared<SEO>();
                seo->setUp(R, Rj, Cj_leg4, C, Vd_seo, 4);
                detection_up.setElement(y, x, seo);
            }
            {
                auto seo = std::make_shared<SEO>();
                seo->setUp(R, Rj, Cj_leg4, C, Vd_seo, 4);
                detection_left.setElement(y, x, seo);
            }
            {
                auto seo = std::make_shared<SEO>();
                seo->setUp(R, Rj, Cj_leg4, C, Vd_seo, 4);
                detection_right.setElement(y, x, seo);
            }
        }
    }

    // oneway回路 (oneway_command_down, oneway_CtoD_down, ...)
    for (int y = 0; y < oneway_command_down.numRows(); ++y) {
        for (int x = 0; x < oneway_command_down.numCols(); ++x) {
            {
                // onway_command_down
                auto unit = std::make_shared<OnewayUnit>("right");
                std::array<std::shared_ptr<BaseElement>, 4> internal_seos;
                for (int i = 0; i < 4; ++i) internal_seos[i] = std::make_shared<SEO>();
                unit->setInternalElements(internal_seos);
                unit->setOnewaySeoParam(R, Rj, Cj_leg2, Cj_leg3, C, Vd_oneway);
                oneway_command_down.setElement(y, x, unit);
            }
            {
                // onway_CtoD_down
                auto unit = std::make_shared<OnewayUnit>("right");
                std::array<std::shared_ptr<BaseElement>, 4> internal_seos;
                for (int i = 0; i < 4; ++i) internal_seos[i] = std::make_shared<SEO>();
                unit->setInternalElements(internal_seos);
                unit->setOnewaySeoParam(R, Rj, Cj_leg2, Cj_leg3, C, Vd_oneway);
                oneway_CtoD_down.setElement(y, x, unit);
            }
            {
                // onway_DtoC_downtoleft
                auto unit = std::make_shared<OnewayUnit>("right");
                std::array<std::shared_ptr<BaseElement>, 4> internal_seos;
                for (int i = 0; i < 4; ++i) internal_seos[i] = std::make_shared<SEO>();
                unit->setInternalElements(internal_seos);
                unit->setOnewaySeoParam(R, Rj, Cj_leg2, Cj_leg3, C, Vd_oneway);
                oneway_DtoC_downtoleft.setElement(y, x, unit);
            }
            {
                // onway_command_left
                auto unit = std::make_shared<OnewayUnit>("right");
                std::array<std::shared_ptr<BaseElement>, 4> internal_seos;
                for (int i = 0; i < 4; ++i) internal_seos[i] = std::make_shared<SEO>();
                unit->setInternalElements(internal_seos);
                unit->setOnewaySeoParam(R, Rj, Cj_leg2, Cj_leg3, C, Vd_oneway);
                oneway_command_left.setElement(y, x, unit);
            }
            {
                // onway_CtoD_left
                auto unit = std::make_shared<OnewayUnit>("right");
                std::array<std::shared_ptr<BaseElement>, 4> internal_seos;
                for (int i = 0; i < 4; ++i) internal_seos[i] = std::make_shared<SEO>();
                unit->setInternalElements(internal_seos);
                unit->setOnewaySeoParam(R, Rj, Cj_leg2, Cj_leg3, C, Vd_oneway);
                oneway_CtoD_left.setElement(y, x, unit);
            }
            {
                // onway_DtoC_lefttoup
                auto unit = std::make_shared<OnewayUnit>("right");
                std::array<std::shared_ptr<BaseElement>, 4> internal_seos;
                for (int i = 0; i < 4; ++i) internal_seos[i] = std::make_shared<SEO>();
                unit->setInternalElements(internal_seos);
                unit->setOnewaySeoParam(R, Rj, Cj_leg2, Cj_leg3, C, Vd_oneway);
                oneway_DtoC_lefttoup.setElement(y, x, unit);
            }
            {
                // onway_command_up
                auto unit = std::make_shared<OnewayUnit>("right");
                std::array<std::shared_ptr<BaseElement>, 4> internal_seos;
                for (int i = 0; i < 4; ++i) internal_seos[i] = std::make_shared<SEO>();
                unit->setInternalElements(internal_seos);
                unit->setOnewaySeoParam(R, Rj, Cj_leg2, Cj_leg3, C, Vd_oneway);
                oneway_command_up.setElement(y, x, unit);
            }
            {
                // onway_CtoD_up
                auto unit = std::make_shared<OnewayUnit>("right");
                std::array<std::shared_ptr<BaseElement>, 4> internal_seos;
                for (int i = 0; i < 4; ++i) internal_seos[i] = std::make_shared<SEO>();
                unit->setInternalElements(internal_seos);
                unit->setOnewaySeoParam(R, Rj, Cj_leg2, Cj_leg3, C, Vd_oneway);
                oneway_CtoD_up.setElement(y, x, unit);
            }
            {
                // onway_DtoC_uptoright
                auto unit = std::make_shared<OnewayUnit>("right");
                std::array<std::shared_ptr<BaseElement>, 4> internal_seos;
                for (int i = 0; i < 4; ++i) internal_seos[i] = std::make_shared<SEO>();
                unit->setInternalElements(internal_seos);
                unit->setOnewaySeoParam(R, Rj, Cj_leg2, Cj_leg3, C, Vd_oneway);
                oneway_DtoC_uptoright.setElement(y, x, unit);
            }
            {
                // onway_command_right
                auto unit = std::make_shared<OnewayUnit>("right");
                std::array<std::shared_ptr<BaseElement>, 4> internal_seos;
                for (int i = 0; i < 4; ++i) internal_seos[i] = std::make_shared<SEO>();
                unit->setInternalElements(internal_seos);
                unit->setOnewaySeoParam(R, Rj, Cj_leg2, Cj_leg3, C, Vd_oneway);
                oneway_command_right.setElement(y, x, unit);
            }
            {
                // onway_CtoD_right
                auto unit = std::make_shared<OnewayUnit>("right");
                std::array<std::shared_ptr<BaseElement>, 4> internal_seos;
                for (int i = 0; i < 4; ++i) internal_seos[i] = std::make_shared<SEO>();
                unit->setInternalElements(internal_seos);
                unit->setOnewaySeoParam(R, Rj, Cj_leg2, Cj_leg3, C, Vd_oneway);
                oneway_CtoD_right.setElement(y, x, unit);
            }
            {
                // onway_DtoC_righttodown
                auto unit = std::make_shared<OnewayUnit>("right");
                std::array<std::shared_ptr<BaseElement>, 4> internal_seos;
                for (int i = 0; i < 4; ++i) internal_seos[i] = std::make_shared<SEO>();
                unit->setInternalElements(internal_seos);
                unit->setOnewaySeoParam(R, Rj, Cj_leg2, Cj_leg3, C, Vd_oneway);
                oneway_DtoC_righttodown.setElement(y, x, unit);
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
                if (y > 0)
                    neighbors.push_back(grid1.getElement(y - 1, x));
                if (x < size_x - 1)
                    neighbors.push_back(grid1.getElement(y, x + 1));
                if (y < size_y - 1)
                    neighbors.push_back(grid1.getElement(y + 1, x));
                if (x > 0)
                    neighbors.push_back(grid1.getElement(y, x - 1));
                neighbors.push_back(grid2.getElement(y, x)); // grid2との接続
                center->setConnections(neighbors);
            }

            {
                auto center = grid2.getElement(y, x);
                std::vector<std::shared_ptr<BaseElement>> neighbors;
                if (y > 0)
                    neighbors.push_back(grid2.getElement(y - 1, x));
                if (x < size_x - 1)
                    neighbors.push_back(grid2.getElement(y, x + 1));
                if (y < size_y - 1)
                    neighbors.push_back(grid2.getElement(y + 1, x));
                if (x > 0)
                    neighbors.push_back(grid2.getElement(y, x - 1));
                neighbors.push_back(grid1.getElement(y, x)); // grid1との接続
                center->setConnections(neighbors);
            }
        }
    }

    // === 接続情報 ===
    // command_down
    std::vector<std::shared_ptr<BaseElement>> neighbors;
    for (int y = 0; y < size_y; ++y) {
        for (int x = 0; x < size_x * particles; ++x) {
            auto elem = command_down.getElement(y, x);
            neighbors.push_back()
            // elem->setConnections(...);
        }
    }
    for (int y = 1; y < size_y; ++y) {
        for (int x = 1; x < size_x * particles; ++x) {
            int particle_id = x % particles;
            int block_x = x / particles;
    
            if (y == 1) {
                // 最初の行
                // 例えば:
                // elem->setConnections({
                //     (右方向衝突判定grid),
                //     (命令方向gridの現在セル),
                //     (左方向衝突判定grid),
                //     (右方向命令grid),
                //     (左方向命令grid)
                // });
            }
            else if (y == size_y - 1) {
                // 最後の行
                // elem->setConnections(6個)
            }
            else {
                // 中間行
                // elem->setConnections(6個)
            }
        }
    }
    

    // detection_down
    for (int y = 0; y < size_y; ++y) {
        for (int x = 0; x < size_x; ++x) {
            auto elem = detection_down.getElement(y, x);
            // elem->setConnections(...);
        }
    }

    // command_left
    for (int y = 0; y < size_y * particles; ++y) {
        for (int x = 0; x < size_x; ++x) {
            auto elem = command_left.getElement(y, x);
            // elem->setConnections(...);
        }
    }

    // detection_left
    for (int y = 0; y < size_y; ++y) {
        for (int x = 0; x < size_x; ++x) {
            auto elem = detection_left.getElement(y, x);
            // elem->setConnections(...);
        }
    }

    // command_up
    for (int y = 0; y < size_y; ++y) {
        for (int x = 0; x < size_x * particles; ++x) {
            auto elem = command_up.getElement(y, x);
            // elem->setConnections(...);
        }
    }

    // detection_up
    for (int y = 0; y < size_y; ++y) {
        for (int x = 0; x < size_x; ++x) {
            auto elem = detection_up.getElement(y, x);
            // elem->setConnections(...);
        }
    }

    // command_right
    for (int y = 0; y < size_y * particles; ++y) {
        for (int x = 0; x < size_x; ++x) {
            auto elem = command_right.getElement(y, x);
            // elem->setConnections(...);
        }
    }

    // detection_right
    for (int y = 0; y < size_y; ++y) {
        for (int x = 0; x < size_x; ++x) {
            auto elem = detection_right.getElement(y, x);
            // elem->setConnections(...);
        }
    }

    // oneway_command_down
    for (int y = 0; y < size_y - 1; ++y) {
        for (int x = 0; x < size_x * particles; ++x) {
            auto elem = oneway_command_down.getElement(y, x);
            // elem->setConnections(...);
        }
    }

    // oneway_CtoD_down
    for (int y = 0; y < size_y - 1; ++y) {
        for (int x = 0; x < size_x * particles; ++x) {
            auto elem = oneway_CtoD_down.getElement(y, x);
            // elem->setConnections(...);
        }
    }

    // oneway_DtoC_downtoleft
    for (int y = 0; y < size_y * particles; ++y) {
        for (int x = 0; x < size_x; ++x) {
            auto elem = oneway_DtoC_downtoleft.getElement(y, x);
            // elem->setConnections(...);
        }
    }

    // oneway_command_left
    for (int y = 0; y < size_y * particles; ++y) {
        for (int x = 0; x < size_x - 1; ++x) {
            auto elem = oneway_command_left.getElement(y, x);
            // elem->setConnections(...);
        }
    }

    // oneway_CtoD_left
    for (int y = 0; y < size_y * particles; ++y) {
        for (int x = 0; x < size_x - 1; ++x) {
            auto elem = oneway_CtoD_left.getElement(y, x);
            // elem->setConnections(...);
        }
    }

    // oneway_DtoC_lefttoup
    for (int y = 0; y < size_y; ++y) {
        for (int x = 0; x < size_x * particles; ++x) {
            auto elem = oneway_DtoC_lefttoup.getElement(y, x);
            // elem->setConnections(...);
        }
    }

    // oneway_command_up
    for (int y = 0; y < size_y - 1; ++y) {
        for (int x = 0; x < size_x * particles; ++x) {
            auto elem = oneway_command_up.getElement(y, x);
            // elem->setConnections(...);
        }
    }

    // oneway_CtoD_up
    for (int y = 0; y < size_y - 1; ++y) {
        for (int x = 0; x < size_x * particles; ++x) {
            auto elem = oneway_CtoD_up.getElement(y, x);
            // elem->setConnections(...);
        }
    }

    // oneway_DtoC_uptoright
    for (int y = 0; y < size_y * particles; ++y) {
        for (int x = 0; x < size_x; ++x) {
            auto elem = oneway_DtoC_uptoright.getElement(y, x);
            // elem->setConnections(...);
        }
    }

    // oneway_command_right
    for (int y = 0; y < size_y * particles; ++y) {
        for (int x = 0; x < size_x - 1; ++x) {
            auto elem = oneway_command_right.getElement(y, x);
            // elem->setConnections(...);
        }
    }

    // oneway_CtoD_right
    for (int y = 0; y < size_y * particles; ++y) {
        for (int x = 0; x < size_x - 1; ++x) {
            auto elem = oneway_CtoD_right.getElement(y, x);
            // elem->setConnections(...);
        }
    }

    // oneway_DtoC_righttodown
    for (int y = 0; y < size_y; ++y) {
        for (int x = 0; x < size_x * particles; ++x) {
            auto elem = oneway_DtoC_righttodown.getElement(y, x);
            // elem->setConnections(...);
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
        grid2.getElement(16, 15)};
    sim.addSelectedElements(ofs, targets);
    std::vector<std::string> labels = {"g1_1515", "g2_1515", "g1_1615", "g2_1615"};
    sim.generateGnuplotScript("../output/multivn.txt", labels);

    // === トリガ設定 ===
    sim.addVoltageTrigger(100, &grid1, 15, 15, 0.006);

    // === 実行 ===
    sim.run();

    // === 動画出力 ===
    const auto &outputs = sim.getOutputs();
    for (const auto &[label, data] : outputs)
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
