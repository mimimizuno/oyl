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
constexpr int size_y = 12;
constexpr double Vd_seo = 0.004;
constexpr double Vd_oneway = 0.0039;
constexpr double R = 1.5;
constexpr double R_small = 0.8;
constexpr double Rj = 0.001;
constexpr double C = 2.0;
constexpr int multi_num = 3;
constexpr double dt = 0.1;
constexpr double endtime = 250;
constexpr double setVth = 0.004;
double multi_cj_leg2 = multi_junction_cj_calc(multi_num, leg2, C, setVth); // 引数の条件に合わせたCjを定義
double multi_cj_leg3 = multi_junction_cj_calc(multi_num, leg3, C, setVth);
double multi_cj_leg4 = multi_junction_cj_calc(multi_num, leg4, C, setVth);
double multi_cj_leg5 = multi_junction_cj_calc(multi_num, leg5, C, setVth);
double multi_cj_leg6 = multi_junction_cj_calc(multi_num, leg6, C, setVth);
constexpr int repeat_count = 1;

using Grid = Grid2D<BaseElement>;
using Sim = Simulation2D<BaseElement>;

int main()
{
    std::vector<std::vector<int>> maze = {
        {0,0,0,0,0,1,0,1,0,1,0,1,0,0,0},
        {0,0,0,0,0,1,0,1,0,1,0,1,0,0,0},
        {1,1,1,1,1,1,0,1,0,1,0,1,0,0,0},
        {0,0,0,0,0,0,0,1,0,1,0,1,0,0,0},
        {1,1,1,1,1,1,1,1,0,1,0,1,0,0,0},
        {0,0,0,0,0,0,0,0,0,1,0,1,0,0,0},
        {1,1,1,1,1,1,1,1,1,1,0,1,0,0,0},
        {0,0,0,0,0,0,0,0,0,0,0,1,0,0,0},
        {1,1,1,1,1,1,1,1,1,1,1,1,0,0,0},
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
    };

    std::vector<std::vector<double>> allResults;
    for(int trial = 0; trial < repeat_count; trial++){
        std::cout << trial << std::endl;
        // === Gridを生成 ===
        Grid command_down(size_y, size_x * particles - 2);                    // 命令方向回路（下）
        Grid detection_down(size_y, size_x);                              // 衝突方向回路（下）
        Grid command_left(size_y * particles - 2, size_x);                    // 命令方向回路（左）
        Grid oneway_command_down(size_y * particles, size_x * particles, false);  // 命令方向回路（下）における一方通行回路
        Grid oneway_CtoD_down(size_y * particles, size_x * particles, false);     // 命令方向回路（下）から衝突判定回路（下）をつなぐ一方通行回路
        Grid oneway_DtoC_downtoleft(size_y * particles, size_x * particles, false);   // 衝突判定回路（下）から命令方向回路（左）をつなぐ一方通行回路
        Grid oneway_command_left(size_y * particles, size_x * particles, false);  // 命令方向回路（左）における一方通行回路

        // 動画出力するgridに名前をつける
        command_down.setOutputLabel("command_down");
        detection_down.setOutputLabel("detection_down");
        command_left.setOutputLabel("command_left");

        // === 全グリッド初期化 ===
        // 命令方向回路 (command_down, command_up)
        for (int y = 0; y < command_down.numRows(); ++y) {
            for (int x = 0; x < command_down.numCols(); ++x) {
                {
                    auto seo = std::make_shared<MultiSEO>();
                    seo->setUp(R, Rj, multi_cj_leg6, C, Vd_seo, leg6, multi_num);
                    command_down.setElement(y, x, seo);
                }
            }
        }
        // 命令方向回路(command_left, command_right)
        for (int y = 0; y < command_left.numRows(); ++y) {
            for (int x = 0; x < command_left.numCols(); ++x) {
                {
                    auto seo = std::make_shared<MultiSEO>();
                    seo->setUp(R, Rj, multi_cj_leg6, C, Vd_seo, leg6, multi_num);
                    command_left.setElement(y, x, seo);
                }
            }
        }

        // 衝突判定回路 (detection_down, detection_up, detection_left, detection_right)
        for (int y = 0; y < detection_down.numRows(); ++y) {
            for (int x = 0; x < detection_down.numCols(); ++x) {
                {
                    auto seo = std::make_shared<MultiSEO>();
                    seo->setUp(R, Rj, multi_cj_leg4, C, Vd_seo, leg4, multi_num);
                    detection_down.setElement(y, x, seo);
                }
            }
        }

        // oneway回路 (oneway_command_down, oneway_CtoD_down, ...)
        for (int y = 0; y < oneway_command_down.numRows(); ++y) {
            for (int x = 0; x < oneway_command_down.numCols(); ++x) {
                {
                    // onway_command_down
                    auto unit = std::make_shared<OnewayUnit>();
                    std::array<std::shared_ptr<BaseElement>, 4> internal_seos;
                    for (int i = 0; i < 4; ++i) internal_seos[i] = std::make_shared<MultiSEO>();
                    unit->setInternalElements(internal_seos);
                    unit->setOnewayMultiSeoParam(R, Rj, multi_cj_leg2, multi_cj_leg3, C, Vd_oneway, multi_num);
                    oneway_command_down.setElement(y, x, unit);
                    if(x > 0 && x < command_down.numCols() && y > 0 && y < command_down.numRows() - 1){
                        unit->setOnewayConnections(command_down.getElement(y,x),command_down.getElement(y+1,x));
                    }
                }
                {
                    // oneway_CtoD_down
                    auto unit = std::make_shared<OnewayUnit>();
                    std::array<std::shared_ptr<BaseElement>, 4> internal_seos;
                    for (int i = 0; i < 4; ++i) internal_seos[i] = std::make_shared<MultiSEO>();
                    unit->setInternalElements(internal_seos);
                    unit->setOnewayMultiSeoParam(R, Rj, multi_cj_leg2, multi_cj_leg3, C, Vd_oneway, multi_num);
                    oneway_CtoD_down.setElement(y, x, unit);
                    if(x > 0 && x < command_down.numCols() && y > 0 && y < command_down.numRows()){
                        int cordinated_x = x / 2;
                        if(x % particles == 1){
                            unit->setOnewayConnections(command_down.getElement(y,x),detection_down.getElement(y,cordinated_x + 1));
                        }
                        else {
                            unit->setOnewayConnections(command_down.getElement(y,x),detection_down.getElement(y,cordinated_x));
                        }
                    }
                }
                {
                    // oneway_DtoC_downtoleft
                    auto unit = std::make_shared<OnewayUnit>();
                    std::array<std::shared_ptr<BaseElement>, 4> internal_seos;
                    for (int i = 0; i < 4; ++i) internal_seos[i] = std::make_shared<MultiSEO>();
                    unit->setInternalElements(internal_seos);
                    unit->setOnewayMultiSeoParam(R, Rj, multi_cj_leg2, multi_cj_leg3, C, Vd_oneway, multi_num);
                    oneway_DtoC_downtoleft.setElement(y, x, unit);
                    if(x > 0 && x < command_left.numCols() && y > 0 && y < command_left.numRows()){
                        int cordinated_y = y / 2;
                        if(y % particles == 1){
                            unit->setOnewayConnections(detection_down.getElement(cordinated_y + 1, x),command_left.getElement(y, x));
                        }
                        else {
                            unit->setOnewayConnections(detection_down.getElement(cordinated_y, x),command_left.getElement(y, x));
                        }
                    }
                }
                {
                    // onway_command_left
                    auto unit = std::make_shared<OnewayUnit>("reverse");
                    std::array<std::shared_ptr<BaseElement>, 4> internal_seos;
                    for (int i = 0; i < 4; ++i) internal_seos[i] = std::make_shared<MultiSEO>();
                    unit->setInternalElements(internal_seos);
                    unit->setOnewayMultiSeoParam(R, Rj, multi_cj_leg2, multi_cj_leg3, C, Vd_oneway, multi_num);
                    oneway_command_left.setElement(y, x, unit);
                    if(x > 0 && x < command_left.numCols() - 1 && y > 0 && y < command_left.numRows()){
                        unit->setOnewayConnections(command_left.getElement(y,x),command_left.getElement(y,x+1));
                    }
                }
            }
        }

        // === 接続情報 ===
        // 命令方向回路 (command_down, command_up)
        for (int y = 1; y < command_down.numRows() - 1; ++y) {
            for (int x = 1; x < command_down.numCols() - 1; ++x) { // xが2倍-2
                int cordinated_x = x / 2;
                int cordinated_y = y * 2; 
                {
                    // command_down
                    auto elem = command_down.getElement(y, x);
                    std::vector<std::shared_ptr<BaseElement>> neighbors;
                    if(x % particles == 1){ // 奇数インデックス
                        // neighbors.push_back(command_right.getElement(cordinated_y, cordinated_x + 1));
                        neighbors.push_back(command_left.getElement(cordinated_y, cordinated_x + 1));
                    }
                    else { // 偶数インデックス
                        // neighbors.push_back(command_right.getElement(cordinated_y - 1, cordinated_x));
                        neighbors.push_back(command_left.getElement(cordinated_y - 1, cordinated_x));
                    }
                    // neighbors.push_back(oneway_DtoC_righttodown.getElement(y,x)->getInternalElement(3)); // 右方向衝突判定
                    neighbors.push_back(oneway_command_down.getElement(y - 1,x)->getInternalElement(3)); // 下方向命令の一方通行（前）
                    neighbors.push_back(oneway_command_down.getElement(y,x)->getInternalElement(0)); // 下方向命令の一方通行（次）
                    neighbors.push_back(oneway_CtoD_down.getElement(y,x)->getInternalElement(0)); // 命令から衝突まで
                    elem->setConnections(neighbors);
                }
            }
        }
        // 命令方向回路(command_left, command_right)
        for (int y = 1; y < command_left.numRows() - 1; ++y) {
            for (int x = 1; x < command_left.numCols() - 1; ++x) {
                int cordinated_x = x * 2;
                int cordinated_y = y / 2; 
                {
                    // command_left
                    auto elem = command_left.getElement(y, x);
                    std::vector<std::shared_ptr<BaseElement>> neighbors;
                    if(y % particles == 1){ // 奇数インデックス
                        // neighbors.push_back(command_up.getElement(cordinated_y + 1, cordinated_x));
                        neighbors.push_back(command_down.getElement(cordinated_y + 1, cordinated_x));
                    }
                    else { // 偶数インデックス
                        // neighbors.push_back(command_up.getElement(cordinated_y, cordinated_x - 1));
                        neighbors.push_back(command_down.getElement(cordinated_y, cordinated_x - 1));
                    }
                    neighbors.push_back(oneway_DtoC_downtoleft.getElement(y,x)->getInternalElement(3)); // 下方向衝突判定
                    neighbors.push_back(oneway_command_left.getElement(y,x - 1)->getInternalElement(3)); // 左方向命令の一方通行（前）
                    neighbors.push_back(oneway_command_left.getElement(y,x)->getInternalElement(0)); // 左方向命令の一方通行（次）
                    // neighbors.push_back(oneway_CtoD_left.getElement(y,x)->getInternalElement(0)); // 命令から衝突まで
                    elem->setConnections(neighbors);
                }
            }
        }
        // 衝突判定回路（detection_down, detection_left, detection_up, detection_right）
        for (int y = 1; y < detection_down.numRows() - 1; ++y) {
            for (int x = 1; x < detection_down.numCols() - 1; ++x) {
                int cordinated_x = x * 2;
                int cordinated_y = y * 2;
                {
                    // detection_down
                    auto elem = detection_down.getElement(y, x);
                    std::vector<std::shared_ptr<BaseElement>> neighbors;
                    neighbors.push_back(oneway_CtoD_down.getElement(y, cordinated_x - 1)->getInternalElement(3)); // 命令から衝突まで
                    neighbors.push_back(oneway_CtoD_down.getElement(y, cordinated_x)->getInternalElement(3));
                    neighbors.push_back(oneway_DtoC_downtoleft.getElement(cordinated_y - 1, x)->getInternalElement(0));
                    neighbors.push_back(oneway_DtoC_downtoleft.getElement(cordinated_y, x)->getInternalElement(0));
                    elem->setConnections(neighbors);
                }
            }
        }

        // バイアス電圧を迷路状に設定
        setMazeBias(command_down,maze,"down",Vd_seo);
        setMazeBias(command_left,maze,"left",Vd_seo);
        setMazeBiasWithDirection_multi(detection_down,maze,"down",Vd_seo, multi_num, multi_cj_leg2, multi_cj_leg3);

        // === シミュレーション初期化 ===
        Sim sim(dt, endtime);
        sim.addGrid({
            command_down, command_left,
            detection_down,
            oneway_command_down, oneway_CtoD_down, oneway_DtoC_downtoleft,
            oneway_command_left, 
        });

        // === 特定素子の出力設定 ===
        std::vector<std::shared_ptr<BaseElement>> tracked = {
            command_down.getElement(3,11),
            command_down.getElement(5,15),
            command_down.getElement(5,16),
            command_down.getElement(7,20),
            command_down.getElement(9,23),
            command_down.getElement(9,24),

            detection_down.getElement(3,6),
            detection_down.getElement(5,8),
            detection_down.getElement(7,10),
            detection_down.getElement(9,12),

            command_left.getElement(5,6),
            command_left.getElement(6,6),
            command_left.getElement(9,8),
            command_left.getElement(13,10),
            command_left.getElement(14,10),
            command_left.getElement(17,12),
            command_left.getElement(18,12),

            command_left.getElement(5,1),
            command_left.getElement(6,1),
            command_left.getElement(9,1),
            command_left.getElement(13,1),
            command_left.getElement(14,1),
            command_left.getElement(17,1),
            command_left.getElement(18,1),
        };
        sim.addTrackedElements(tracked);

        // === トリガ設定 ===
        sim.addVoltageTrigger(150, &command_down, 1, 11, 0.002);
        sim.addVoltageTrigger(150, &command_down, 1, 15, 0.002);
        sim.addVoltageTrigger(150, &command_down, 1, 16, 0.002);
        sim.addVoltageTrigger(150, &command_down, 1, 20, 0.002);
        sim.addVoltageTrigger(150, &command_down, 1, 23, 0.002);
        sim.addVoltageTrigger(150, &command_down, 1, 24, 0.002);

        // === 実行 ===
        sim.run();
        // 時刻記録
        allResults.push_back(sim.getTunnelTimes());
    }

    // === CSV出力 ===
    std::ofstream ofs("../output/tunnel_times_summary.csv");
    for (const auto& row : allResults) {
        for (size_t i = 0; i < row.size(); ++i) {
            ofs << row[i];
            if (i != row.size() - 1) ofs << ",";
        }
        ofs << "\n";
    }

    return 0;
}
