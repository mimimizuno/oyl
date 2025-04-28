#ifndef SIMULATION_2D_HPP
#define SIMULATION_2D_HPP

#include <fstream>
#include <vector>
#include <memory>
#include <utility>
#include <map>
#include <cmath>
#include "seo_class.hpp"
#include "grid_2dim.hpp"
// #include "output_class.hpp"

template <typename Element>
class Simulation2D
{
private:
    double t;                           // 現在の時間（不定期に増える）
    double dt;                          // 基本刻み（参考値）
    double endtime;                     // 終了時刻
    double outputInterval;              // 出力間隔（例: 0.1）
    double nextOutputTime;              // 次に出力すべき時刻（0.1, 0.2, ...）
    std::vector<Grid2D<Element>> grids; // Grid2Dのインスタンス配列
    // oyl-video形式のデータ
    std::map<
        std::string,                                  // ラベル名
        std::vector<std::vector<std::vector<double>>> // [timeframe][y][x]
        >
        outputs;
    // トリガを表すベクトル（どのgridか、時刻、位置、値)
    std::vector<std::tuple<Grid2D<Element>*,double, int, int, double>> voltageTriggers; // (grid, time, x, y, V)

public:
    // コンストラクタ(刻み時間,シミュレーションの終了タイミング)
    Simulation2D(double dT, double EndTime);

    // wtの比較（gridごと）
    std::pair<bool, std::shared_ptr<Grid2D<Element>>> comparewt();

    // トンネルの処理
    void handleTunnels(Grid2D<Element> &tunnelgrid);

    // ファイル作成
    void openFiles() const;

    // ファイル閉じる
    void closeFiles() const;

    // ファイル出力
    void outputToFile();

    // oyl-video形式に合わせた出力を生成
    void outputTooyl();

    // シミュレーションの1ステップ
    void runStep();

    // シミュレーションにgridを追加
    void addGrid(const std::vector<Grid2D<Element>> &Gridinstance);

    // シミュレーションの実行
    void run();

    // グリッド取得
    std::vector<Grid2D<Element>> &getGrids();

    // outputsを取得
    const std::map<std::string, std::vector<std::vector<std::vector<double>>>> &getOutputs() const;

    // トリガーを追加する
    void addVoltageTrigger(double triggerTime, Grid2D<Element>* grid, int x, int y, double voltage);

    // トリガを適用させる
    void applyVoltageTriggers();

};

// コンストラクタ
template <typename Element>
Simulation2D<Element>::Simulation2D(double dT, double EndTime)
    : t(0.0), dt(dT), endtime(EndTime), outputInterval(dT), nextOutputTime(0.0) {}

// 最小wtをもつgridを探索する（最小wtがdtより小さいかどうかのbool, 最小のwtを持つgrid）
template <typename Element>
std::pair<bool, std::shared_ptr<Grid2D<Element>>> Simulation2D<Element>::comparewt()
{
    double minwt = dt;
    std::shared_ptr<Grid2D<Element>> tunnelgrid = nullptr;
    for (auto &grid : grids)
    {
        if (grid.gridminwt(dt))
        {
            double candidate = grid.getMinWT();
            if (candidate < minwt)
            {
                minwt = candidate;
                tunnelgrid = std::make_shared<Grid2D<Element>>(grid);
            }
        }
    }
    if (minwt < dt) return {true, tunnelgrid};
    return {false, nullptr};
}


// トンネル処理を実行
template <typename Element>
void Simulation2D<Element>::handleTunnels(Grid2D<Element> &tunnelgrid)
{
    auto ptr = tunnelgrid.getTunnelPlace();
    //----------------トンネル場所の記録--------------------------------
    auto [y, x] = tunnelgrid.getPositionOf(ptr);
    std::ofstream log("output/tunnel_log.txt", std::ios::app);
    if (log.is_open()) {
        log << "t=" << t
            << ", x=" << x
            << ", y=" << y
            << ", dir=" << tunnelgrid.getTunnelDirection()
            << std::endl;
    }
    //-----------------------------------------------------------------
    // 実際のトンネル処理
    ptr->setTunnel(tunnelgrid.getTunnelDirection());
}

// ファイルを開く
template <typename Element>
void Simulation2D<Element>::openFiles() const
{
    // for (auto &outputdata : printdatavector)
    // {
    //     outputdata.openFile();
    // }
}

// ファイルを閉じる
template <typename Element>
void Simulation2D<Element>::closeFiles() const
{
    // for (auto &outputdata : printdatavector)
    // {
    //     outputdata.closeFile();
    // }
}

// 出力処理（未実装部分を仮追加）
template <typename Element>
void Simulation2D<Element>::outputToFile()
{
    // if (accumulatedTime >= outputInterval)
    // {
    //     accumulatedTime -= outputInterval;
    //     // TODO: 各 printdatavector に対してデータを渡す処理を入れる
    // }
}

// oyl-video形式に合わせた出力を生成
// 1 <= x <= max-1, 1 <= y <= max-1の範囲で出力される(sizex=32,sizey=32の場合は1から31までの範囲で30×30になる)
template <typename Element>
void Simulation2D<Element>::outputTooyl()
{
    if (t >= nextOutputTime)
    {
        // 出力形式に合わせて整数値にならす
        int timeframe = static_cast<int>(std::round(nextOutputTime / outputInterval));
        int outputIndex = 0; // 出力順にindex付けするカウンタ

        for (const auto &grid : grids)
        {
            if (!grid.isOutputEnabled())
                continue;

            std::string label;
            if (grid.hasOutputLabel())
            {
                label = grid.getOutputLabel();
            }
            else
            {
                label = "output" + std::to_string(outputIndex);
                ++outputIndex;
            }

            int rows = grid.numRows();
            int cols = grid.numCols();

            std::vector<std::vector<double>> vnGrid(rows - 2, std::vector<double>(cols - 2));
            for (int i = 1; i < rows - 1; ++i)
            {
                for (int j = 1; j < cols - 1; ++j)
                {
                    auto elem = grid.getElement(i, j);
                    double vn = elem->getVn();
                    double vd = elem->getVd();
            
                    // Vdが負のとき、Vnを反転して記録
                    if (vd < 0) {
                        vn *= -1.0;
                    }
            
                    vnGrid[i-1][j-1] = vn;
                }
            }            

            outputs[label].resize(timeframe + 1);
            outputs[label][timeframe] = vnGrid;
        }
        nextOutputTime += outputInterval;
    }
}

// シミュレーションの1ステップを実行
template <typename Element>
void Simulation2D<Element>::runStep()
{
    double steptime = dt;

    // oyl-video形式に出力
    outputTooyl();

    // grid全体のVn計算(5回計算してならす)
    for (int i = 0; i < 5; i++)
    {
        for (auto &grid : grids)
        {
            // 接続されている電圧を更新
            grid.updateGridSurVn();
            // トリガの適用
            applyVoltageTriggers();
            // 電圧を更新
            grid.updateGridVn();
        }
    }

    // grid全体のdE計算
    for (auto &grid : grids)
    {
        grid.updateGriddE();
    }

    // wtの計算と比較
    auto compared = this->comparewt();
    if (compared.first)
    {
        handleTunnels(*compared.second);
        steptime = compared.second->getMinWT();
    }

    // チャージの計算
    for (auto &grid : grids)
    {
        grid.updateGridQn(steptime);
    }

    // tの増加
    t += steptime;
}

// Gridインスタンスの配列を登録
template <typename Element>
void Simulation2D<Element>::addGrid(const std::vector<Grid2D<Element>> &Gridinstance)
{
    grids = Gridinstance;
}

// 全体シミュレーションの実行
template <typename Element>
void Simulation2D<Element>::run()
{
    // openFiles();
    // トンネルの場所を記録するファイルの出力（別関数にする予定）

    while (t < endtime)
    {
        runStep();
    }
    // closeFiles();
}

// グリッド取得
template <typename Element>
std::vector<Grid2D<Element>> &Simulation2D<Element>::getGrids()
{
    return grids;
}

template <typename Element>
const std::map<std::string, std::vector<std::vector<std::vector<double>>>> &Simulation2D<Element>::getOutputs() const
{
    return outputs;
}

template <typename Element>
void Simulation2D<Element>::addVoltageTrigger(double triggerTime, Grid2D<Element>* grid, int x, int y, double voltage) {
    voltageTriggers.emplace_back(grid, triggerTime, x, y, voltage);
}

template <typename Element>
void Simulation2D<Element>::applyVoltageTriggers()
{
    for (const auto& [gridPtr, triggerTime, x, y, voltage] : voltageTriggers) {
        if (t >= triggerTime && t < triggerTime + dt) {
            if (!gridPtr) {
                throw std::invalid_argument("Trigger references a null grid pointer.");
            }

            if (x < 0 || x >= gridPtr->numCols() || y < 0 || y >= gridPtr->numRows()) {
                throw std::out_of_range(
                    "Trigger coordinates (x=" + std::to_string(x) +
                    ", y=" + std::to_string(y) + ") are out of grid bounds (" +
                    std::to_string(gridPtr->numCols()) + "x" + std::to_string(gridPtr->numRows()) + ")."
                );
            }

            auto elem = gridPtr->getElement(y, x);
            elem->setVsum(elem->getSurroundingVsum() + voltage);
        }
    }
}

#endif // SIMULATION_2D_HPP