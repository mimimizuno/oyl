// maze_bias.hpp
#ifndef PARTICLE_COMPUTATION_METHODS_HPP
#define PARTICLE_COMPUTATION_METHODS_HPP

#include "grid_2dim.hpp"
#include "constants.hpp"
#include <vector>
#include <stdexcept>

// maze example
// std::vector<std::vector<int>> maze = {
//     {0,0,0,0,0,0,0,1,0,1,0,1,0,1,0},
//     {0,0,1,0,0,0,0,1,0,1,0,1,0,1,0},
//     {0,1,1,1,1,0,0,1,0,1,0,1,0,1,0},
//     {0,1,1,1,1,1,1,1,1,1,1,1,1,1,0},
//     {0,0,1,0,1,0,0,0,0,0,0,1,0,1,0},
//     {0,0,1,0,1,0,0,1,0,0,0,1,0,1,0},
//     {0,0,1,0,1,0,1,1,1,1,0,1,0,1,0},
//     {0,0,1,0,1,0,1,1,1,1,1,1,1,1,0},
//     {0,0,1,0,1,0,0,1,0,1,0,0,0,0,0},
// };

// 命令方向回路にバイアス電圧を設定する関数
// 与えられた迷路ベクトルの0（壁）1（通路)を読み取ってgridの要素にVdを設定する
template<typename Element>
void setMazeBias(Grid2D<Element>& grid, const std::vector<std::vector<int>>& maze, const std::string& direction, double Vd_normal, double Vd_wall = 0.0) {
    int rows = grid.numRows();
    int cols = grid.numCols();

    // 方向によってmazeとgridのマッピングの仕方が異なる
    bool vertical_expand = (direction == "left" || direction == "right"); // 左右の場合、縦が2倍

    // mazeのサイズチェック
    if (vertical_expand) {
        if (maze.size() != static_cast<size_t>((rows - 2) / 2)) {
            throw std::invalid_argument("Maze height must match (grid rows - 2) / particles for left/right direction.");
        }
        for (const auto& row : maze) {
            if (row.size() != static_cast<size_t>(cols - 2)) {
                throw std::invalid_argument("Maze width must match (grid cols - 2) for left/right direction.");
            }
        }
    } else {
        if (maze.size() != static_cast<size_t>(rows - 2)) {
            throw std::invalid_argument("Maze height must match (grid rows - 2) for up/down direction.");
        }
        for (const auto& row : maze) {
            if (row.size() != static_cast<size_t>((cols - 2) / 2)) {
                throw std::invalid_argument("Maze width must match (grid cols - 2) / particles for up/down direction.");
            }
        }
    }

    for (int y = 0; y < rows; ++y) {
        for (int x = 0; x < cols; ++x) {
            auto elem = grid.getElement(y, x);

            // 端は無条件で壁電圧
            if (x == 0 || y == 0 || x == cols - 1 || y == rows - 1) {
                elem->setVias(Vd_wall);
                continue;
            }

            bool isPassage = false;

            if (vertical_expand) { // left or right (縦が2倍)
                int maze_y = (y - 1) / 2;
                int maze_x = x - 1;
                if (maze_y >= 0 && maze_y < static_cast<int>(maze.size()) && maze_x >= 0 && maze_x < static_cast<int>(maze[0].size())) {
                    isPassage = (maze[maze_y][maze_x] == 1);
                }
            } else { // up or down (横が2倍)
                int maze_y = y - 1;
                int maze_x = (x - 1) / 2;
                if (maze_y >= 0 && maze_y < static_cast<int>(maze.size()) && maze_x >= 0 && maze_x < static_cast<int>(maze[0].size())) {
                    isPassage = (maze[maze_y][maze_x] == 1);
                }
            }

            if (isPassage) {
                elem->setVias(Vd_normal);
            } else {
                elem->setVias(Vd_wall);
            }
        }
    }
}

// 与えられた迷路ベクトルの0（壁）1（通路)を読み取ってgridの要素にVdを設定。上下左右の方向を読み取り、障害物の１マス手前を低めの値で設定する関数
// 衝突判定回路にバイアス電圧を設定する関数
// 壁の1マス手前にVd_normal、2マス手前にVd_lowerを設定
template<typename Element>
void setMazeBiasWithDirection(
    Grid2D<Element>& grid,
    const std::vector<std::vector<int>>& maze,
    const std::string& direction,
    double Vd_normal,
    double c = 2.0,
    double cj_leg2 = Cj_leg2,
    double cj_leg3 = Cj_leg3,
    double Vd_wall = 0.0
) {
    double Vd_lower = Vd_normal - ((c * e) / ((3 * c + cj_leg3) * (2 * c + cj_leg2)));
    int rows = grid.numRows();
    int cols = grid.numCols();

    if (maze.size() != static_cast<size_t>(rows - 2) || maze[0].size() != static_cast<size_t>(cols - 2)) {
        throw std::invalid_argument("Maze size must match grid size - 2.");
    }

    // まずすべてを壁に初期化
    for (int y = 0; y < rows; ++y)
        for (int x = 0; x < cols; ++x)
            grid.getElement(y, x)->setVias(Vd_wall);

    for (int y = 1; y < rows - 1; ++y) {
        for (int x = 1; x < cols - 1; ++x) {
            int maze_y = y - 1;
            int maze_x = x - 1;

            if (maze[maze_y][maze_x] != 1) continue; // 通路だけ対象

            bool hasWall = false;
            if (direction == "up" && maze_y > 0 && maze[maze_y - 1][maze_x] == 0) {
                hasWall = true;
                grid.getElement(y, x)->setVias(Vd_normal);
                if (y + 1 < rows - 1 && maze[maze_y + 1][maze_x] == 1) grid.getElement(y + 1, x)->setVias(Vd_lower);
            } else if (direction == "down" && maze_y < maze.size() - 1 && maze[maze_y + 1][maze_x] == 0) {
                hasWall = true;
                grid.getElement(y, x)->setVias(Vd_normal);
                if (y - 1 > 0 && maze[maze_y - 1][maze_x] == 1) grid.getElement(y - 1, x)->setVias(Vd_lower);
            } else if (direction == "left" && maze_x > 0 && maze[maze_y][maze_x - 1] == 0) {
                hasWall = true;
                grid.getElement(y, x)->setVias(Vd_normal);
                if (x + 1 < cols - 1 && maze[maze_y][maze_x + 1] == 1) grid.getElement(y, x + 1)->setVias(Vd_lower);
            } else if (direction == "right" && maze_x < maze[0].size() - 1 && maze[maze_y][maze_x + 1] == 0) {
                hasWall = true;
                grid.getElement(y, x)->setVias(Vd_normal);
                if (x - 1 > 0 && maze[maze_y][maze_x - 1] == 1) grid.getElement(y, x - 1)->setVias(Vd_lower);
            }
        }
    }
}

// 衝突判定回路　多重トンネル用
template<typename Element>
void setMazeBiasWithDirection_multi(
    Grid2D<Element>& grid,
    const std::vector<std::vector<int>>& maze,
    const std::string& direction,
    double Vd_normal,
    int multi_num,
    double cj_leg2,
    double cj_leg3,
    double c = 2.0,
    double ratio = 0.8,
    double Vd_wall = 0.0
) {
    double Vd_lower = Vd_normal - ratio * ((c * multi_num * multi_num * e) / ((leg3 * multi_num * c + cj_leg3) * (leg3 * multi_num * c + cj_leg2)));
    int rows = grid.numRows();
    int cols = grid.numCols();

    if (maze.size() != static_cast<size_t>(rows - 2) || maze[0].size() != static_cast<size_t>(cols - 2)) {
        throw std::invalid_argument("Maze size must match grid size - 2.");
    }

    // まずすべてを壁に初期化
    for (int y = 0; y < rows; ++y)
        for (int x = 0; x < cols; ++x)
            grid.getElement(y, x)->setVias(Vd_wall);

    for (int y = 1; y < rows - 1; ++y) {
        for (int x = 1; x < cols - 1; ++x) {
            int maze_y = y - 1;
            int maze_x = x - 1;

            if (maze[maze_y][maze_x] != 1) continue; // 通路だけ対象

            bool hasWall = false;
            if (direction == "up" && maze_y > 0 && maze[maze_y - 1][maze_x] == 0) {
                hasWall = true;
                grid.getElement(y, x)->setVias(Vd_normal);
                if (y + 1 < rows - 1 && maze[maze_y + 1][maze_x] == 1) grid.getElement(y + 1, x)->setVias(Vd_lower);
            } else if (direction == "down" && maze_y < maze.size() - 1 && maze[maze_y + 1][maze_x] == 0) {
                hasWall = true;
                grid.getElement(y, x)->setVias(Vd_normal);
                if (y - 1 > 0 && maze[maze_y - 1][maze_x] == 1) grid.getElement(y - 1, x)->setVias(Vd_lower);
            } else if (direction == "left" && maze_x > 0 && maze[maze_y][maze_x - 1] == 0) {
                hasWall = true;
                grid.getElement(y, x)->setVias(Vd_normal);
                if (x + 1 < cols - 1 && maze[maze_y][maze_x + 1] == 1) grid.getElement(y, x + 1)->setVias(Vd_lower);
            } else if (direction == "right" && maze_x < maze[0].size() - 1 && maze[maze_y][maze_x + 1] == 0) {
                hasWall = true;
                grid.getElement(y, x)->setVias(Vd_normal);
                if (x - 1 > 0 && maze[maze_y][maze_x - 1] == 1) grid.getElement(y, x - 1)->setVias(Vd_lower);
            }
        }
    }
}


#endif // PARTICLE_COMPUTATION_METHODS_HPP