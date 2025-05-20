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

// 対角接続の伝搬を確認する

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
                if(x > 0 && x < command_down.numCols() && y > 0 && y < command_down.numRows() - 1){
                    unit->setOnewayConnections(command_down.getElement(y,x),command_down.getElement(y+1,x));
                }
            }
            {
                // oneway_CtoD_down
                auto unit = std::make_shared<OnewayUnit>("right");
                std::array<std::shared_ptr<BaseElement>, 4> internal_seos;
                for (int i = 0; i < 4; ++i) internal_seos[i] = std::make_shared<SEO>();
                unit->setInternalElements(internal_seos);
                unit->setOnewaySeoParam(R, Rj, Cj_leg2, Cj_leg3, C, Vd_oneway);
                oneway_CtoD_down.setElement(y, x, unit);
                if(x > 0 && x < command_down.numCols() && y > 0 && y < command_down.numRows()){
                    int cordinated_x = x / 2;
                    if(x % particles == 1){
                        unit->setOnewayConnections(command_down.getElement(y,x),detection_down.getElement(y,cordinated_x));
                    }
                    else {
                        unit->setOnewayConnections(command_down.getElement(y,x),detection_down.getElement(y,cordinated_x + 1));
                    }
                }
            }
            {
                // oneway_DtoC_downtoleft
                auto unit = std::make_shared<OnewayUnit>("right");
                std::array<std::shared_ptr<BaseElement>, 4> internal_seos;
                for (int i = 0; i < 4; ++i) internal_seos[i] = std::make_shared<SEO>();
                unit->setInternalElements(internal_seos);
                unit->setOnewaySeoParam(R, Rj, Cj_leg2, Cj_leg3, C, Vd_oneway);
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
                auto unit = std::make_shared<OnewayUnit>("left");
                std::array<std::shared_ptr<BaseElement>, 4> internal_seos;
                for (int i = 0; i < 4; ++i) internal_seos[i] = std::make_shared<SEO>();
                unit->setInternalElements(internal_seos);
                unit->setOnewaySeoParam(R, Rj, Cj_leg2, Cj_leg3, C, Vd_oneway);
                oneway_command_left.setElement(y, x, unit);
                if(x > 0 && x < command_left.numCols() - 1 && y > 0 && y < command_left.numRows()){
                    unit->setOnewayConnections(command_left.getElement(y,x),command_left.getElement(y,x+1));
                }
            }
            {
                // onway_CtoD_left
                auto unit = std::make_shared<OnewayUnit>("right");
                std::array<std::shared_ptr<BaseElement>, 4> internal_seos;
                for (int i = 0; i < 4; ++i) internal_seos[i] = std::make_shared<SEO>();
                unit->setInternalElements(internal_seos);
                unit->setOnewaySeoParam(R, Rj, Cj_leg2, Cj_leg3, C, Vd_oneway);
                oneway_CtoD_left.setElement(y, x, unit);
                if(x > 0 && x < command_left.numCols() && y > 0 && y < command_left.numRows()){
                    int cordinated_y = y / 2;
                    if(y % particles == 1){
                        unit->setOnewayConnections(command_left.getElement(y,x),detection_left.getElement(cordinated_y, x));
                    }
                    else {
                        unit->setOnewayConnections(command_left.getElement(y,x),detection_left.getElement(cordinated_y + 1, x));
                    }
                }
            }
            {
                // oneway_DtoC_lefttoup
                auto unit = std::make_shared<OnewayUnit>("right");
                std::array<std::shared_ptr<BaseElement>, 4> internal_seos;
                for (int i = 0; i < 4; ++i) internal_seos[i] = std::make_shared<SEO>();
                unit->setInternalElements(internal_seos);
                unit->setOnewaySeoParam(R, Rj, Cj_leg2, Cj_leg3, C, Vd_oneway);
                oneway_DtoC_lefttoup.setElement(y, x, unit);
                if(x > 0 && x < command_up.numCols() && y > 0 && y < command_up.numRows()){
                    int cordinated_x = x / 2;
                    if(x % particles == 1){
                        unit->setOnewayConnections(detection_left.getElement(y, cordinated_x + 1),command_up.getElement(y, x));
                    }
                    else {
                        unit->setOnewayConnections(detection_left.getElement(y, cordinated_x),command_up.getElement(y, x));
                    }
                }
            }
            {
                // oneway_command_up
                auto unit = std::make_shared<OnewayUnit>("left");
                std::array<std::shared_ptr<BaseElement>, 4> internal_seos;
                for (int i = 0; i < 4; ++i) internal_seos[i] = std::make_shared<SEO>();
                unit->setInternalElements(internal_seos);
                unit->setOnewaySeoParam(R, Rj, Cj_leg2, Cj_leg3, C, Vd_oneway);
                oneway_command_up.setElement(y, x, unit);
                if(x > 0 && x < command_up.numCols() && y > 0 && y < command_up.numRows() - 1){
                    unit->setOnewayConnections(command_up.getElement(y,x),command_up.getElement(y + 1,x));
                }
            }
            {
                // oneway_CtoD_up
                auto unit = std::make_shared<OnewayUnit>("right");
                std::array<std::shared_ptr<BaseElement>, 4> internal_seos;
                for (int i = 0; i < 4; ++i) internal_seos[i] = std::make_shared<SEO>();
                unit->setInternalElements(internal_seos);
                unit->setOnewaySeoParam(R, Rj, Cj_leg2, Cj_leg3, C, Vd_oneway);
                oneway_CtoD_up.setElement(y, x, unit);
                if(x > 0 && x < command_up.numCols() && y > 0 && y < command_up.numRows()){
                    int cordinated_x = x / 2;
                    if(x % particles == 1){
                        unit->setOnewayConnections(command_down.getElement(y,x),detection_down.getElement(y,cordinated_x));
                    }
                    else {
                        unit->setOnewayConnections(command_down.getElement(y,x),detection_down.getElement(y,cordinated_x + 1));
                    }
                }
            }
            {
                // onway_DtoC_uptoright
                auto unit = std::make_shared<OnewayUnit>("right");
                std::array<std::shared_ptr<BaseElement>, 4> internal_seos;
                for (int i = 0; i < 4; ++i) internal_seos[i] = std::make_shared<SEO>();
                unit->setInternalElements(internal_seos);
                unit->setOnewaySeoParam(R, Rj, Cj_leg2, Cj_leg3, C, Vd_oneway);
                oneway_DtoC_uptoright.setElement(y, x, unit);
                if(x > 0 && x < command_right.numCols() && y > 0 && y < command_right.numRows()){
                    int cordinated_y = y / 2;
                    if(y % particles == 1){
                        unit->setOnewayConnections(detection_up.getElement(cordinated_y + 1, x),command_right.getElement(y, x));
                    }
                    else {
                        unit->setOnewayConnections(detection_up.getElement(cordinated_y, x),command_right.getElement(y, x));
                    }
                }
            }
            {
                // onway_command_right
                auto unit = std::make_shared<OnewayUnit>("right");
                std::array<std::shared_ptr<BaseElement>, 4> internal_seos;
                for (int i = 0; i < 4; ++i) internal_seos[i] = std::make_shared<SEO>();
                unit->setInternalElements(internal_seos);
                unit->setOnewaySeoParam(R, Rj, Cj_leg2, Cj_leg3, C, Vd_oneway);
                oneway_command_right.setElement(y, x, unit);
                if(x > 0 && x < command_right.numCols() - 1 && y > 0 && y < command_right.numRows()){
                    unit->setOnewayConnections(command_right.getElement(y,x),command_right.getElement(y,x + 1));
                }
            }
            {
                // onway_CtoD_right
                auto unit = std::make_shared<OnewayUnit>("right");
                std::array<std::shared_ptr<BaseElement>, 4> internal_seos;
                for (int i = 0; i < 4; ++i) internal_seos[i] = std::make_shared<SEO>();
                unit->setInternalElements(internal_seos);
                unit->setOnewaySeoParam(R, Rj, Cj_leg2, Cj_leg3, C, Vd_oneway);
                oneway_CtoD_right.setElement(y, x, unit);
                if(x > 0 && x < command_right.numCols() && y > 0 && y < command_right.numRows()){
                    int cordinated_y = y / 2;
                    if(y % particles == 1){
                        unit->setOnewayConnections(command_right.getElement(y,x),detection_right.getElement(cordinated_y, x));
                    }
                    else {
                        unit->setOnewayConnections(command_right.getElement(y,x),detection_right.getElement(cordinated_y + 1, x));
                    }
                }
            }
            {
                // onway_DtoC_righttodown
                auto unit = std::make_shared<OnewayUnit>("right");
                std::array<std::shared_ptr<BaseElement>, 4> internal_seos;
                for (int i = 0; i < 4; ++i) internal_seos[i] = std::make_shared<SEO>();
                unit->setInternalElements(internal_seos);
                unit->setOnewaySeoParam(R, Rj, Cj_leg2, Cj_leg3, C, Vd_oneway);
                oneway_DtoC_righttodown.setElement(y, x, unit);
                if(x > 0 && x < command_down.numCols() && y > 0 && y < command_down.numRows()){
                    int cordinated_x = x / 2;
                    if(x % particles == 1){
                        unit->setOnewayConnections(detection_right.getElement(y, cordinated_x + 1),command_down.getElement(y, x));
                    }
                    else {
                        unit->setOnewayConnections(detection_right.getElement(y, cordinated_x),command_down.getElement(y, x));
                    }
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
                    neighbors.push_back(command_right.getElement(cordinated_y, cordinated_x + 1));
                    neighbors.push_back(command_left.getElement(cordinated_y, cordinated_x + 1));
                }
                else { // 偶数インデックス
                    neighbors.push_back(command_right.getElement(cordinated_y - 1, cordinated_x));
                    neighbors.push_back(command_left.getElement(cordinated_y - 1, cordinated_x));
                }
                neighbors.push_back(oneway_DtoC_righttodown.getElement(y,x)->getInternalElement(3)); // 右方向衝突判定
                neighbors.push_back(oneway_command_down.getElement(y - 1,x)->getInternalElement(3)); // 下方向命令の一方通行（前）
                neighbors.push_back(oneway_command_down.getElement(y,x)->getInternalElement(0)); // 下方向命令の一方通行（次）
                neighbors.push_back(oneway_CtoD_down.getElement(y,x)->getInternalElement(0)); // 命令から衝突まで
                elem->setConnections(neighbors);
            }
            {
                // command_up
                auto elem = command_up.getElement(y, x);
                std::vector<std::shared_ptr<BaseElement>> neighbors;
                if(x % particles == 1){ // 奇数インデックス
                    neighbors.push_back(command_right.getElement(cordinated_y, cordinated_x + 1));
                    neighbors.push_back(command_left.getElement(cordinated_y, cordinated_x + 1));
                }
                else { // 偶数インデックス
                    neighbors.push_back(command_right.getElement(cordinated_y - 1, cordinated_x));
                    neighbors.push_back(command_left.getElement(cordinated_y - 1, cordinated_x));
                }
                neighbors.push_back(oneway_DtoC_lefttoup.getElement(y,x)->getInternalElement(3)); // 左方向衝突判定
                neighbors.push_back(oneway_command_up.getElement(y - 1,x)->getInternalElement(3)); // 上方向命令の一方通行（前）
                neighbors.push_back(oneway_command_up.getElement(y,x)->getInternalElement(0)); // 上方向命令の一方通行（次）
                neighbors.push_back(oneway_CtoD_up.getElement(y,x)->getInternalElement(0)); // 命令から衝突まで
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
                    neighbors.push_back(command_up.getElement(cordinated_y, cordinated_x + 1));
                    neighbors.push_back(command_down.getElement(cordinated_y, cordinated_x + 1));
                }
                else { // 偶数インデックス
                    neighbors.push_back(command_up.getElement(cordinated_y, cordinated_x - 1));
                    neighbors.push_back(command_down.getElement(cordinated_y - 1, cordinated_x - 1));
                }
                neighbors.push_back(oneway_DtoC_downtoleft.getElement(y,x)->getInternalElement(3)); // 下方向衝突判定
                neighbors.push_back(oneway_command_left.getElement(y - 1,x)->getInternalElement(3)); // 左方向命令の一方通行（前）
                neighbors.push_back(oneway_command_left.getElement(y,x)->getInternalElement(0)); // 左方向命令の一方通行（次）
                neighbors.push_back(oneway_CtoD_left.getElement(y,x)->getInternalElement(0)); // 命令から衝突まで
                elem->setConnections(neighbors);
            }
            {
                // command_right
                auto elem = command_right.getElement(y, x);
                std::vector<std::shared_ptr<BaseElement>> neighbors;
                if(x % particles == 1){ // 奇数インデックス
                    neighbors.push_back(command_up.getElement(cordinated_y, cordinated_x + 1));
                    neighbors.push_back(command_down.getElement(cordinated_y, cordinated_x + 1));
                }
                else { // 偶数インデックス
                    neighbors.push_back(command_up.getElement(cordinated_y, cordinated_x - 1));
                    neighbors.push_back(command_down.getElement(cordinated_y, cordinated_x - 1));
                }
                neighbors.push_back(oneway_DtoC_uptoright.getElement(y,x)->getInternalElement(3)); // 右方向衝突判定
                neighbors.push_back(oneway_command_right.getElement(y - 1,x)->getInternalElement(3)); // 下方向命令の一方通行（前）
                neighbors.push_back(oneway_command_right.getElement(y,x)->getInternalElement(0)); // 下方向命令の一方通行（次）
                neighbors.push_back(oneway_CtoD_right.getElement(y,x)->getInternalElement(0)); // 命令から衝突まで
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
            {
                // detection_left
                auto elem = detection_left.getElement(y, x);
                std::vector<std::shared_ptr<BaseElement>> neighbors;
                neighbors.push_back(oneway_CtoD_left.getElement(cordinated_y - 1, x)->getInternalElement(3)); // 命令から衝突まで
                neighbors.push_back(oneway_CtoD_left.getElement(cordinated_y, x)->getInternalElement(3));
                neighbors.push_back(oneway_DtoC_lefttoup.getElement(y, cordinated_x - 1)->getInternalElement(0));
                neighbors.push_back(oneway_DtoC_lefttoup.getElement(y, cordinated_x)->getInternalElement(0));
                elem->setConnections(neighbors);
            }
            {
                // detection_up
                auto elem = detection_up.getElement(y, x);
                std::vector<std::shared_ptr<BaseElement>> neighbors;
                neighbors.push_back(oneway_CtoD_up.getElement(y, cordinated_x - 1)->getInternalElement(3)); // 命令から衝突まで
                neighbors.push_back(oneway_CtoD_up.getElement(y, cordinated_x)->getInternalElement(3));
                neighbors.push_back(oneway_DtoC_uptoright.getElement(cordinated_y - 1, x)->getInternalElement(0));
                neighbors.push_back(oneway_DtoC_uptoright.getElement(cordinated_y, x)->getInternalElement(0));
                elem->setConnections(neighbors);
            }
            {
                // detection_right
                auto elem = detection_right.getElement(y, x);
                std::vector<std::shared_ptr<BaseElement>> neighbors;
                neighbors.push_back(oneway_CtoD_right.getElement(cordinated_y - 1, x)->getInternalElement(3)); // 命令から衝突まで
                neighbors.push_back(oneway_CtoD_right.getElement(cordinated_y, x)->getInternalElement(3));
                neighbors.push_back(oneway_DtoC_righttodown.getElement(y, cordinated_x - 1)->getInternalElement(0));
                neighbors.push_back(oneway_DtoC_righttodown.getElement(y, cordinated_x)->getInternalElement(0));
                elem->setConnections(neighbors);
            }
        }
    }

    // バイアス電圧を迷路状に設定
    setMazeBias(command_down,maze,"down",Vd_seo);
    setMazeBias(command_left,maze,"left",Vd_seo);
    setMazeBias(command_up,maze,"up",Vd_seo);
    setMazeBias(command_right,maze,"right",Vd_seo);
    setMazeBiasWithDirection(detection_down,maze,"down",Vd_seo);
    setMazeBiasWithDirection(detection_left,maze,"left",Vd_seo);
    setMazeBiasWithDirection(detection_up,maze,"up",Vd_seo);
    setMazeBiasWithDirection(detection_right,maze,"right",Vd_seo);

    // === シミュレーション初期化 ===
    Sim sim(dt, endtime);
    sim.addGrid({
        command_down, command_left, command_up, command_right,
        detection_down, detection_left, detection_up, detection_right,
        oneway_command_down, oneway_CtoD_down, oneway_DtoC_downtoleft,
        oneway_command_left, oneway_CtoD_left, oneway_DtoC_lefttoup,
        oneway_command_up, oneway_CtoD_up, oneway_DtoC_uptoright,
        oneway_command_right, oneway_CtoD_right, oneway_DtoC_righttodown
    });

    // === 特定素子の出力設定 ===
    auto ofs1 = std::make_shared<std::ofstream>("../output/seos_down.txt");
    std::vector<std::shared_ptr<BaseElement>> targets1 = {
        command_down.getElement(1, 1),
        command_down.getElement(1, 16),
        command_down.getElement(1, 20),
        command_down.getElement(1, 24),
        command_down.getElement(1, 28)};
    sim.addSelectedElements(ofs1, targets1);
    std::vector<std::string> labels1 = {"down1-1", "down1-16", "down1-20", "down1-24", "down1-28"};
    sim.generateGnuplotScript("../output/seos_down.txt", labels1);

    auto ofs2 = std::make_shared<std::ofstream>("../output/seos_up.txt");
    std::vector<std::shared_ptr<BaseElement>> targets2 = {
        command_up.getElement(1, 1),
        command_up.getElement(1, 16),
        command_up.getElement(1, 20),
        command_up.getElement(1, 24),
        command_up.getElement(1, 28)};
    sim.addSelectedElements(ofs2, targets2);
    std::vector<std::string> labels2 = {"up1-1", "up1-16", "up1-20", "up1-24", "up1-28"};
    sim.generateGnuplotScript("../output/seos_up.txt", labels2);

    auto ofs3 = std::make_shared<std::ofstream>("../output/oneway.txt");
    std::vector<std::shared_ptr<BaseElement>> targets3 = {
        oneway_command_down.getElement(1,16)->getInternalElement(0),
        oneway_command_down.getElement(1,16)->getInternalElement(1),
        oneway_command_down.getElement(1,16)->getInternalElement(2),
        oneway_command_down.getElement(1,16)->getInternalElement(3),
    };
    sim.addSelectedElements(ofs3, targets3);
    std::vector<std::string> labels3 = {"oneway-d0", "oneway-d1", "oneway-d2", "oneway-d3"};
    sim.generateGnuplotScript("../output/oneway.txt", labels3);

    // === トリガ設定 ===
    sim.addVoltageTrigger(100, &command_down, 1, 16, 0.0006);
    sim.addVoltageTrigger(100, &command_down, 1, 24, 0.0006);

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
