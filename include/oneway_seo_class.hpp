#ifndef ONEWAY_UNIT_HPP
#define ONEWAY_UNIT_HPP

#include "constants.hpp"
#include <memory>
#include <array>
#include <vector>
#include <string>
#include <map>
#include <cmath>
#include <stdexcept>

// 一方向伝導素子を表すテンプレートクラス
// ElementはSEO, MultiSEOなどを想定

template <typename Element>
class OnewayUnit
{
private:
    std::array<std::shared_ptr<Element>, 4> ows; // 一方通行のための4つの素子を用意
    std::string oneway_direction;                // 一方通行の方向("left"3から0の方向,"right"0から3の方向)
    std::shared_ptr<Element> locate = nullptr;   // 最小wtを持つ素子
    std::string tunnel_direction = "none";       // トンネルの方向を保持

public:
    // コンストラクタ：一方通行の向きと4つの素子を初期化（デフォルトは右向き。左向きにしたい場合は引数にleftを指定）
    OnewayUnit(std::string onewaydirection = "right") : oneway_direction(onewaydirection)
    {
        if (onewaydirection != "left" && onewaydirection != "right")
        {
            throw std::invalid_argument("OnewayUnit: direction must be \"left\" or \"right\"");
        }
        for (auto &elem : ows)
        {
            elem = std::make_shared<Element>();
        }
    }

    // onway_directionの設定
    void setOnewayDirection(const std::string &onewaydirection)
    {
        if (onewaydirection != "left" && onewaydirection != "right")
        {
            throw std::invalid_argument("direction must be \"left\" or \"right\"");
        }
        oneway_direction = onewaydirection;
    }

    // 一方通行の中身の素子(seo)にパラメータを付与(R,Rj,Cj_leg2,Cj_leg3,C,Vd)
    void setOnewaySeoParam(double r, double rj, double cj_leg2, double cj_leg3, double c, double vd)
    {
        double vias = -vd + ((c * e) / ((3 * c + cj_leg3) * (2 * c + cj_leg2)));
        for (int i = 0; i < 4; i++)
        {
            if (i == 0 || i == 3)
            {
                ows[i]->setUp(r, rj, cj_leg3, c, -vd, 3);
            }
            else
            {
                ows[i]->setUp(r, rj, cj_leg2, c, vd, 2);
            }
        }
        if (oneway_direction == "right")
            ows[3]->setVias(vias);
        else
            ows[0]->setVias(vias);
    }

    // 一方通行の中身の素子(multiseo)にパラメータを付与(R,Rj,Cj_leg2,Cj_leg3,C,Vd,multi_num)
    void setOnewaySeoParam(double r, double rj, double cj_leg2, double cj_leg3, double c, double vd, int junction_num)
    {
        double vias = -vd + ((c * e) / ((3 * c + cj_leg3) * (2 * c + cj_leg2)));
        for (int i = 0; i < 4; i++)
        {
            if (i == 0 || i == 3)
            {
                ows[i]->setUp(r, rj, cj_leg3, c, -vd, 3, junction_num);
            }
            else
            {
                ows[i]->setUp(r, rj, cj_leg2, c, vd, 2, junction_num);
            }
        }
        if (oneway_direction == "right")
            ows[3]->setVias(vias);
        else
            ows[0]->setVias(vias);
    }

    // 両端の素子を含むconnectionの設定
    void setOnewayConnections(std::shared_ptr<Element> left_elem, std::shared_ptr<Element> right_elem)
    {
        for (int i = 0; i < 4; i++)
        {
            std::vector<std::shared_ptr<Element>> connections;
            if (i == 0)
            {
                connections.push_back(left_elem);
                connections.push_back(ows[1]);
                connections.push_back(ows[2]);
            }
            else if (i == 1 || i == 2)
            {
                connections.push_back(ows[0]);
                connections.push_back(ows[3]);
            }
            else
            {
                connections.push_back(right_elem);
                connections.push_back(ows[1]);
                connections.push_back(ows[2]);
            }
            ows[i]->setConnections(connections);
        }
    }

    // getVnなど、定義しないとgrid_2dimなどでエラーが発生するため素子0を返す
    // 出力電位Vnを返す（代表として素子0を利用）
    double getVn() const
    {
        return ows[0]->getVn();
    }

    // 印加電圧Vdを返す（代表として素子0を利用）
    double getVd() const
    {
        return ows[0]->getVd();
    }

    // 接続素子の周辺電圧を設定
    void setSurroundingVoltages()
    {
        for (auto &e : ows)
            e->setSurroundingVoltages();
    }

    // 接続素子の確率計算を実行
    void setPcalc()
    {
        for (auto &e : ows)
            e->setPcalc();
    }

    // 接続素子のエネルギー差を計算
    void setdEcalc()
    {
        for (auto &e : ows)
            e->setdEcalc();
    }

    // 最小トンネル待ち時間を計算し、対象素子と方向を記録
    bool calculateTunnelWt()
    {
        double min_wt = 1e9;
        bool found = false;
        for (auto &e : ows)
        {
            if (e->calculateTunnelWt())
            {
                auto wt_map = e->getWT();
                for (const auto &entry : wt_map)
                {
                    if (entry.second < min_wt)
                    {
                        min_wt = entry.second;
                        locate = e;
                        tunnel_direction = entry.first;
                        found = true;
                    }
                }
            }
        }
        return found;
    }

    // 最小トンネル素子のWT（待ち時間）を返す（locateが存在しなければ空のmapを返す）
    std::map<std::string, double> getWT() const
    {
        return locate ? locate->getWT() : std::map<std::string, double>{};
    }

    // 全ての素子に対して電荷の更新処理を行う
    void setNodeCharge(double dt)
    {
        for (auto &e : ows)
            e->setNodeCharge(dt);
    }

    // 最小wt素子に対してトンネルを設定
    void setTunnel()
    {
        if (locate)
        {
            locate->setTunnel(tunnel_direction);
        }
    }

    // 記録されたトンネル方向を返す
    std::string getTunnelDirection() const
    {
        return tunnel_direction;
    }

    // 内部の素子群を取得（デバッグや出力用）
    const std::array<std::shared_ptr<Element>, 4>& getInternalElements() const {
        return ows;
    }
};

#endif // ONEWAY_UNIT_HPP
