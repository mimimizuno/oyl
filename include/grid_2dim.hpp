#ifndef GRID_2DIM_HPP
#define GRID_2DIM_HPP

#include <vector>
#include <memory>
#include <stdexcept>
#include <algorithm>
#include <string>
#include <fstream>
#include "base_element.hpp"

// 2次元グリッドで任意の素子（Element）を管理するテンプレートクラス
template <typename Element>
class Grid2D
{
private:
    // 2次元gridの定義
    std::vector<std::vector<std::shared_ptr<Element>>> grid;
    // 縦横のサイズ
    int rows_, cols_;
    // 出力時のファイル名(デフォルトは空)
    std::string outputlabel = "";
    // 電子トンネルをする場所
    std::shared_ptr<Element> tunnelplace;
    // 電子トンネルの向き
    std::string tunneldirection;
    // gridにおける最小の待ち時間
    double minwt;
    // 出力するかのbool値(デフォルトがtrueで出力する)
    bool outputEnabled;
public:
    // コンストラクタ：指定した行数・列数でグリッドを初期化
    Grid2D(int rows, int cols, bool enableOutput = true); // ← outputするかどうかのbool。デフォルトをtrueにする

    // 指定位置の要素を取得
    std::shared_ptr<Element> getElement(int row, int col) const;

    // 指定位置の要素を設定
    void setElement(int row, int col, const std::shared_ptr<Element> &element);

    // グリッド全体の接続されている電圧を更新
    void updateGridSurVn();

    // グリッド全体のノード電圧Vnを計算・更新
    void updateGridVn();

    // グリッド全体のエネルギー変化dEを計算・更新
    void updateGriddE();

    // グリッド全体のトンネル待ち時間wtを計算し、最小wtとトンネル素子を更新
    bool gridminwt(const double dt);

    // グリッド全体のノード電荷Qnを更新
    void updateGridQn(const double dt);

    // グリッド（全体の2次元vector）を取得
    std::vector<std::vector<std::shared_ptr<Element>>> &getGrid();

    // 行数を取得
    int numRows() const;

    // 列数を取得
    int numCols() const;

    // トンネルが発生する素子を取得
    std::shared_ptr<Element> getTunnelPlace() const;

    // トンネルの方向（"up" or "down"）を取得
    std::string getTunnelDirection() const;

    // 最小トンネル待ち時間wtを取得
    double getMinWT() const;

    // outputlabelの設定
    void setOutputLabel(const std::string& label);

    // outputlabelの取得
    std::string getOutputLabel() const;

    // outputlabelが設定されているかの取得
    bool hasOutputLabel() const;

    // OutputEnabledの設定
    void setOutputEnabled(bool flag);

    // OutputEnabledの取得
    bool isOutputEnabled() const;

    // ポインタから場所の座標を取得する
    std::pair<int, int> getPositionOf(const std::shared_ptr<Element>& ptr) const;
};

// コンストラクタ：全要素をmake_sharedで初期化
template <typename Element>
Grid2D<Element>::Grid2D(int rows, int cols, bool enableOutput)
    : rows_(rows), cols_(cols), grid(rows, std::vector<std::shared_ptr<Element>>(cols)),
      outputEnabled(enableOutput)
{
    if (rows <= 0 || cols <= 0)
    {
        throw std::invalid_argument("Grid size must be positive");
    }
}

// 指定位置の要素を取得
template <typename Element>
std::shared_ptr<Element> Grid2D<Element>::getElement(int row, int col) const
{
    return grid.at(row).at(col);
}

// 指定位置の要素を設定
template <typename Element>
void Grid2D<Element>::setElement(int row, int col, const std::shared_ptr<Element> &element)
{
    grid.at(row).at(col) = element;
}

// グリッド全体の接続されている電圧を更新
template <typename Element>
void Grid2D<Element>::updateGridSurVn()
{
    for (auto &row : grid)
    {
        for (auto &elem : row)
        {
            elem->setSurroundingVoltages();
        }
    }
}

// グリッド全体のノード電圧Vnを計算・更新
template <typename Element>
void Grid2D<Element>::updateGridVn()
{
    for (auto &row : grid)
    {
        for (auto &elem : row)
        {
            elem->setPcalc();
        }
    }
}

// グリッド全体のエネルギー変化dEを計算・更新
template <typename Element>
void Grid2D<Element>::updateGriddE()
{
    for (auto &row : grid)
    {
        for (auto &elem : row)
        {
            elem->setdEcalc();
        }
    }
}

// グリッド全体のトンネル待ち時間wtを計算し、最小wtとトンネル素子・方向を記録
template <typename Element>
bool Grid2D<Element>::gridminwt(const double dt)
{
    minwt = dt;
    for (auto &row : grid)
    {
        for (auto &elem : row)
        {
            if (elem->calculateTunnelWt())
            {
                // up方向かdown方向で値を持っている方をtmpwtに代入
                double tmpwt = std::max(elem->getWT()["up"], elem->getWT()["down"]);
                // tmpwtがminwtよりも値が小さい時にminwtを更新
                if(tmpwt < minwt){
                    tunneldirection = (tmpwt == elem->getWT()["up"]) ? "up" : "down";
                    tunnelplace = elem;
                    minwt = std::min(minwt, tmpwt);
                }
            }
        }
    }
    return minwt < dt;
}

// グリッド全体のノード電荷Qnを計算・更新
template <typename Element>
void Grid2D<Element>::updateGridQn(const double dt)
{
    for (auto &row : grid)
    {
        for (auto &elem : row)
        {
            elem->setNodeCharge(dt);
        }
    }
}

// グリッド全体のデータを取得
template <typename Element>
std::vector<std::vector<std::shared_ptr<Element>>> &Grid2D<Element>::getGrid()
{
    return grid;
}

// グリッドの行数を取得
template <typename Element>
int Grid2D<Element>::numRows() const
{
    return rows_;
}

// グリッドの列数を取得
template <typename Element>
int Grid2D<Element>::numCols() const
{
    return cols_;
}

// 最小wtでトンネルが発生する素子を取得
template <typename Element>
std::shared_ptr<Element> Grid2D<Element>::getTunnelPlace() const
{
    return tunnelplace;
}

// トンネルの方向を取得（"up" または "down"）
template <typename Element>
std::string Grid2D<Element>::getTunnelDirection() const
{
    return tunneldirection;
}

// 最小トンネル待ち時間wtを取得
template <typename Element>
double Grid2D<Element>::getMinWT() const
{
    return minwt;
}

// outputlabelの設定
template <typename Element>
void Grid2D<Element>::setOutputLabel(const std::string& label)
{
    outputlabel = label;
}


// outputlabelの取得
template <typename Element>
std::string Grid2D<Element>::getOutputLabel() const
{
    return outputlabel.empty() ? "" : outputlabel;
}

// outputlabelが設定されているかの取得
template <typename Element>
bool Grid2D<Element>::hasOutputLabel() const
{
    return !outputlabel.empty();
}

// outputEnabledにbool値を設定
template <typename Element>
void Grid2D<Element>::setOutputEnabled(bool flag)
{
    outputEnabled = flag;
}

// OutputEnabledを取得
template <typename Element>
bool Grid2D<Element>::isOutputEnabled() const
{
    return outputEnabled;
}

// ポインタから場所の座標を取得する
template <typename Element>
std::pair<int, int> Grid2D<Element>::getPositionOf(const std::shared_ptr<Element>& ptr) const {
    for (int i = 0; i < rows_; ++i) {
        for (int j = 0; j < cols_; ++j) {
            if (grid[i][j] == ptr) {
                return {i, j}; // (y, x)
            }
        }
    }
    throw std::runtime_error("Element pointer not found in grid");
}

#endif // GRID_2DIM_HPP
