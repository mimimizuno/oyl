#ifndef ONEWAY_UNIT_HPP
#define ONEWAY_UNIT_HPP

#include "constants.hpp"
#include "base_element.hpp"
#include "seo_class.hpp"
#include "multi_seo_class.hpp"
#include <memory>
#include <array>
#include <vector>
#include <string>
#include <map>
#include <cmath>
#include <stdexcept>

// 一方向伝導素子を表すクラス（BaseElementに対応）
class OnewayUnit : public BaseElement
{
private:
    std::array<std::shared_ptr<BaseElement>, 4> ows; // 一方通行のための4つの素子を用意
    std::string oneway_direction;                      // 一方通行の方向("left"3から0の方向,"right"0から3の方向)
    std::shared_ptr<BaseElement> locate = nullptr;     // 最小wtを持つ素子
    std::string tunnel_direction = "none";            // トンネルの方向を保持

public:
    // コンストラクタ：一方通行の向きと4つの素子を初期化（デフォルトは右向き。左向きにしたい場合は引数にleftを指定）
    OnewayUnit(std::string onewaydirection = "right") : oneway_direction(onewaydirection)
    {
        if (onewaydirection != "left" && onewaydirection != "right")
        {
            throw std::invalid_argument("OnewayUnit: direction must be \"left\" or \"right\"");
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

    // 一方通行の内部素子を手動設定（SEOやMultiSEOを入れる）
    void setInternalElements(const std::array<std::shared_ptr<BaseElement>, 4> &elements)
    {
        ows = elements;
    }

    // 一方通行の中身の素子(seo)にパラメータを付与(R,Rj,Cj_leg2,Cj_leg3,C,Vd)
    void setOnewaySeoParam(double r, double rj, double cj_leg2, double cj_leg3, double c, double vd)
    {
        double vias = -vd + ((c * e) / ((3 * c + cj_leg3) * (2 * c + cj_leg2)));
        for (int i = 0; i < 4; i++)
        {
            auto ptr = std::dynamic_pointer_cast<SEO>(ows[i]);
            if (i == 0 || i == 3)
            {
                ptr->setUp(r, rj, cj_leg3, c, -vd, 3);
            }
            else
            {
                ptr->setUp(r, rj, cj_leg2, c, vd, 2);
            }
        }
        if (oneway_direction == "right")
            std::dynamic_pointer_cast<SEO>(ows[3])->setVias(vias);
        else
            std::dynamic_pointer_cast<SEO>(ows[0])->setVias(vias);
    }

    // 一方通行の中身の素子(multiseo)にパラメータを付与(R,Rj,Cj_leg2,Cj_leg3,C,Vd,multi_num)
    void setOnewayMultiSeoParam(double r, double rj, double cj_leg2, double cj_leg3, double c, double vd, int junction_num)
    {
        double vias = -vd + ((c * e) / ((3 * c + cj_leg3) * (2 * c + cj_leg2)));
        for (int i = 0; i < 4; i++)
        {
            auto ptr = std::dynamic_pointer_cast<MultiSEO>(ows[i]);
            if (i == 0 || i == 3)
            {
                ptr->setUp(r, rj, cj_leg3, c, -vd, 3, junction_num);
            }
            else
            {
                ptr->setUp(r, rj, cj_leg2, c, vd, 2, junction_num);
            }
        }
        if (oneway_direction == "right")
            std::dynamic_pointer_cast<MultiSEO>(ows[3])->setVias(vias);
        else
            std::dynamic_pointer_cast<MultiSEO>(ows[0])->setVias(vias);
    }
    // 両端の素子を含むconnectionの設定
    void setOnewayConnections(std::shared_ptr<BaseElement> left_elem, std::shared_ptr<BaseElement> right_elem)
    {
        for (int i = 0; i < 4; i++)
        {
            std::vector<std::shared_ptr<BaseElement>> connections;
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
    double getVn() const override { return ows[0]->getVn(); }

    // 印加電圧Vdを返す（代表として素子0を利用）
    double getVd() const override { return ows[0]->getVd(); }

    // 接続素子の周辺電圧を設定
    void setSurroundingVoltages() override
    {
        for (auto &e : ows)
            e->setSurroundingVoltages();
    }

    // 接続素子の確率計算を実行
    void setPcalc() override
    {
        for (auto &e : ows)
            e->setPcalc();
    }

    // 接続素子のエネルギー差を計算
    void setdEcalc() override
    {
        for (auto &e : ows)
            e->setdEcalc();
    }

    // 最小トンネル待ち時間を計算し、対象素子と方向を記録
    bool calculateTunnelWt() override
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
    std::map<std::string, double> getWT() const override
    {
        return locate ? locate->getWT() : std::map<std::string, double>{};
    }

    // 全ての素子に対して電荷の更新処理を行う
    void setNodeCharge(double dt) override
    {
        for (auto &e : ows)
            e->setNodeCharge(dt);
    }

    // 最小wt素子に対してトンネルを設定
    void setTunnel(const std::string &direction) override
    {
        if (locate)
        {
            locate->setTunnel(direction);
        }
    }

    // 記録されたトンネル方向を返す
    std::string getTunnelDirection() const
    {
        return tunnel_direction;
    }

    // 内部の素子群を取得（デバッグや出力用）
    const std::array<std::shared_ptr<BaseElement>, 4> &getInternalElements() const
    {
        return ows;
    }

    // 周辺電圧合計を取得（代表素子に委譲）
    double getSurroundingVsum() const override
    {
        return ows[0]->getSurroundingVsum();
    }

    // Vsum設定（各素子に一括）
    void setVsum(double v) override
    {
        for (auto &e : ows)
            e->setVsum(v);
    }

    // 接続を一括で設定
    void setConnections(const std::vector<std::shared_ptr<BaseElement>> &conns) override
    {
        for (auto &e : ows)
            e->setConnections(conns);
    }

    // setViasをオーバーライドするためのダミー関数
    void seVias(const double vd){
    }
};

#endif // ONEWAY_UNIT_HPP