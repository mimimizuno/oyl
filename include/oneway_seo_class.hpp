#ifndef ONEWAY_UNIT_HPP
#define ONEWAY_UNIT_HPP

#include "constants.hpp"
#include <memory>
#include <array>
#include <vector>
#include <string>
#include <map>
#include <cmath>

// 一方向伝導素子を表すテンプレートクラス
// ElementはSEO, MultiSEOなどを想定

template <typename Element>
class OnewayUnit {
private:
    std::array<std::shared_ptr<Element>, 4> ows;            // 一方通行のための4つの素子を用意
    std::string oneway_direction;                           // 一方通行の方向("left"3から0の方向,"right"0から3の方向)
    // double Vd;                                              // バイアス電圧
    // double R;                                               // 抵抗
    // double Rj;                                              // トンネル抵抗
    // double C;                                               // 接続容量
    // double Cj_leg2;                                         // 接合容量(足が2本の素子)
    // double Cj_leg3;                                         // 接合容量(足が3本の素子)
    // int multi_num = 0;                                      // ジャンクションの数（多重の数）
    std::shared_ptr<Element> locate = nullptr;              // 最小wtを持つ素子
    std::string tunnel_direction = "none";                  // トンネルの方向を保持
    // std::array<std::shared_ptr<Element>,2> connected_elems; // 一方通行の両端の素子に接続される素子

public:
    // コンストラクタ：4つの素子を初期化
    OnewayUnit() {
        for (auto &elem : ows) {
            elem = std::make_shared<Element>();
        }
    }

    // 一方通行の中身の素子(seo)にパラメータを付与(Vd,R,Rj,C,Cj_leg2,Cj_leg3)
    void setOnewaySeoParam(){}

    // 一方通行の中身の素子(multiseo)にパラメータを付与(Vd,R,Rj,C,Cj_leg2,Cj_leg3, multi_num)
    void setOnewaySeoParam(){}

    // 両端の素子を含むconnectionの設定
    void setOnewayConnections(std::shared_ptr<Element> left_elem, std::shared_ptr<Element> right_elem){
        for(int i = 0; i < 4; i++){
            std::vector<std::shared_ptr<Element>> connections;
            if(i == 0){
                connections.push_back(left_elem);
                connections.push_back(ows[1]);
                connections.push_back(ows[2]);
            }
            else if(i == 1 || i == 2){
                connections.push_back(ows[0]);
                connections.push_back(ows[3]);
            }
            else{
                connections.push_back(right_elem);
                connections.push_back(ows[1]);
                connections.push_back(ows[2]);
            }
            ows[i]->setConnections(connections);
        }
    }

    // getVnなど、定義しないとgrid_2dimなどでエラーが発生するため素子0を返す
    // 出力電位Vnを返す（代表として素子0を利用）
    double getVn() const {
        return ows[0]->getVn();
    }

    // 印加電圧Vdを返す（代表として素子0を利用）
    double getVd() const {
        return ows[0]->getVd();
    }

    // 接続素子の周辺電圧を設定
    void setSurroundingVoltages() {
        for (auto &e : ows) e->setSurroundingVoltages();
    }

    // 接続素子の確率計算を実行
    void setPcalc() {
        for (auto &e : ows) e->setPcalc();
    }

    // 接続素子のエネルギー差を計算
    void setdEcalc() {
        for (auto &e : ows) e->setdEcalc();
    }

    // 最小トンネル待ち時間を計算し、対象素子と方向を記録
    bool calculateTunnelWt() {
        double min_wt = 1e9;
        bool found = false;
        for (auto &e : ows) {
            if (e->calculateTunnelWt()) {
                auto wt_map = e->getWT();
                for (const auto &entry : wt_map) {
                    if (entry.second < min_wt) {
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
    std::map<std::string, double> getWT() const {
        return locate ? locate->getWT() : std::map<std::string, double>{};
    }

    // 全ての素子に対して電荷の更新処理を行う
    void setNodeCharge(double dt) {
        for (auto &e : ows) e->setNodeCharge(dt);
    }

    // 最小wt素子に対してトンネルを設定
    void setTunnel() {
        if (locate) {
            locate->setTunnel(tunnel_direction);
        }
    }

    // 記録されたトンネル方向を返す
    std::string getTunnelDirection() const {
        return tunnel_direction;
    }

    // 各素子にVdと素子の情報をセット。
    void setOnewayVd() {
        for (int i = 0; i < 4; i++) {
            if (i == 1 || i == 2) {
                ows[i]->setVias(Vd);
            } else if (i == 0 && oneway_direction == "left") {
                ows[i]->setVias(-Vd + ((C * e) / ((3 * C + Cjs_leg3) * (2 * C + Cj_leg2))));
            } else if (i == 3 && oneway_direction == "left") {
                ows[i]->setVias(-Vd);
            } else if (i == 0 && oneway_direction == "right") {
                ows[i]->setVias(-Vd);
            } else if (i == 3 && oneway_direction == "right") {
                ows[i]->setVias(-Vd + ((C * e) / ((3 * C + Cjs_leg3) * (2 * C + Cj_leg2))));
            }
        }
    }

};

#endif // ONEWAY_UNIT_HPP
