#include "memory_class.hpp"
//------ コンストラクタ（パラメータの初期設定）---------//
// 初期値無し
MEMORY::MEMORY() : Rj(0), Cj(0), C(0), Vd(0), Vn(0), legs(0), V_sum(0), upper_electrons(0), lower_electrons(0)
{
    dE["up1"] = 0.0;
    dE["up2"] = 0.0;
    dE["down1"] = 0.0;
    dE["down2"] = 0.0;
    wt["up1"] = 0.0;
    wt["up2"] = 0.0;
    wt["down1"] = 0.0;
    wt["down2"] = 0.0;
}

// 初期値あり
MEMORY::MEMORY(double rj, double cj, double c, double cl, double vd, int legscounts)
    : Rj(rj), Cj(cj), C(c), Vd(vd), Vn(0.0), legs(legscounts), V_sum(0.0), upper_electrons(0), lower_electrons(0)
{
    dE["up1"] = 0.0;
    dE["up2"] = 0.0;
    dE["down1"] = 0.0;
    dE["down2"] = 0.0;
    wt["up1"] = 0.0;
    wt["up2"] = 0.0;
    wt["down1"] = 0.0;
    wt["down2"] = 0.0;
}

//-----------セッター------------//
// パラメータセットアップ
void MEMORY::setUp(double rj, double cj, double c, double cl, double vd, int legscounts)
{
    Rj = rj;
    Cj = cj;
    C = c;
    Vd = vd;
    legs = legscounts;
    CL = cl;
}

// バイアス電圧を設定
void MEMORY::setVias(const double vd)
{
    Vd = vd;
}

// V_sumを設定
void MEMORY::setVsum(double v)
{
    V_sum = v;
}

// 接続情報を設定
void MEMORY::setConnections(const std::vector<std::shared_ptr<BaseElement>>& conns) {
    connections.clear();
    if (conns.size() > legs) {
        throw std::invalid_argument("Too many connections for the number of legs.");
    }
    for (const auto& elem : conns) {
        if (elem.get() == this) {
            throw std::invalid_argument("Cannot connect to itself.");
        }
        connections.push_back(elem);
    }
}

// 周囲の電圧を設定
void MEMORY::setSurroundingVoltages()
{
    V_sum = 0;
    for (auto elem : connections)
    {
        V_sum += elem->getVn();
    }
}

// メモリのパラメータ計算
void MEMORY::setPcalc()
{
    Vn = (Cj * e * ( 2 * upper_electrons + lower_electrons) + 2 * Cj * CL * Vd + 2 * C * Cj * V_sum) / (Cj * (2 * legs * C + Cj + 2 * CL));
}

// メモリのエネルギー計算
void MEMORY::setdEcalc()
{
    dE["up1"] = -e * (CL * e * (1 + 2 * lower_electrons) + Cj * (e + 2 * e * (lower_electrons + upper_electrons) + 2 * CL * Vd) + C * (legs * e + 2 * legs * e * lower_electrons + 2 * Cj * V_sum)) / (2 * Cj * (2 * legs * C + Cj + 2 * CL)); // lower_electrons + 1
    dE["up2"] = -e * (CL * e * (1 - 2 * lower_electrons) + Cj * (e + 2 * e * upper_electrons + 2 * CL * Vd) + C * (legs * e - 2 * legs * e * lower_electrons + 2 * Cj * V_sum)) / (2 * Cj * (2 * legs * C + Cj + 2 * CL)); // upper_electrons + 1, lower_electrons - 1
    dE["down1"] = -e * (CL * e * (1 + 2 * lower_electrons) + Cj * (e - 2 * e * upper_electrons - 2 * CL * Vd) + C * (legs * e + 2 * legs * e * lower_electrons - 2 * Cj * V_sum)) / (2 * Cj * (2 * legs * C + Cj + 2 * CL)); // lower_electrons + 1, upper_electrons - 1
    dE["down2"] = -e * (CL * e * (1 - 2 * lower_electrons) + Cj * (e - 2 * e * (lower_electrons + upper_electrons) - 2 * CL * Vd) + C * (legs * e - 2 * legs * e * lower_electrons - 2 * Cj * V_sum)) / (2 * Cj * (2 * legs * C + Cj + 2 * CL)); // lower_electrons - 1
}

// 電荷の更新(メモリは電荷の更新を持たないため、呼び出すとエラー)
void MEMORY::setNodeCharge(const double dt)
{
    throw std::runtime_error("This element does not have setNodeCharge function");
}

// トンネル待ち時間計算(up1, up2, down1, down2のいずれかが正の時にwtを計算してtrueを返す)
bool MEMORY::calculateTunnelWt()
{
    // 初期化
    wt["up1"] = 0;
    wt["up2"] = 0;
    wt["down1"] = 0;
    wt["down2"] = 0;
    if (dE["up1"] > 0)
    {
        wt["up1"] = (e * e * Rj / dE["up1"]) * std::log(1 / Random());
        return true;
    }
    if (dE["up2"] > 0)
    {
        wt["up2"] = (e * e * Rj / dE["up2"]) * std::log(1 / Random());
        return true;
    }
        if (dE["down1"] > 0)
    {
        wt["down1"] = (e * e * Rj / dE["down1"]) * std::log(1 / Random());
        return true;
    }
    if (dE["down2"] > 0)
    {
        wt["down2"] = (e * e * Rj / dE["down2"]) * std::log(1 / Random());
        return true;
    }
    return false;
}

// メモリのトンネル
void MEMORY::setTunnel(const std::string& direction)
{
    if (direction == "up1")
    {
        lower_electrons++;
    }
    else if (direction == "up2")
    {
        lower_electrons--;
        upper_electrons++;
    }
        if (direction == "down1")
    {
        lower_electrons++;
        upper_electrons--;
    }
    else if (direction == "down2")
    {
        lower_electrons--;
    }
}
//-----------ゲッター------------//

// ノード電圧を取得
double MEMORY::getVn() const
{
    return Vn;
}

// 接続されてる振動子の電圧の総和を取得
double MEMORY::getSurroundingVsum() const
{
    return V_sum;
}

// dEの取得(up1,up2,down1,down2)
std::map<std::string, double> MEMORY::getdE() const
{
    return dE;
}

// // Qの取得
// double SEO::getQ() const
// {
//     return Q;
// }

// wtの取得(up1,up2,down1,down2)
std::map<std::string, double> MEMORY::getWT() const
{
    return wt;
}

// oneway用の関数のため呼び出すとエラー発生
std::shared_ptr<BaseElement> MEMORY::getInternalElement(int index) const 
{
    throw std::runtime_error("This element does not have internal elements.");
}

//-------- 汎用処理 -------------//
// 0から1の間の乱数を生成
double MEMORY::Random()
{
    static std::random_device rd;
    static std::mt19937 mt(rd());
    static std::uniform_real_distribution<double> dist(0.0, 1.0);
    return dist(mt);
}

//-------- テスト用 -----------//
// テスト用Rjゲッター
double MEMORY::getRj() const
{
    return Rj;
}

// テスト用Cjゲッター
double MEMORY::getCj() const
{
    return Cj;
}

// テスト用Cゲッター
double MEMORY::getC() const
{
    return C;
}

// テスト用Vdゲッター
double MEMORY::getVd() const
{
    return Vd;
}

// テスト用legsゲッター
int MEMORY::getlegs() const
{
    return legs;
}

// テスト用dEセッター
void MEMORY::setdE(const std::string &direction, double value)
{
    dE[direction] = value;
}

// テスト用Vnセッター
void MEMORY::setVn(double vn)
{
    Vn = vn;
}

// // テスト用Qnセッター
// void SEO::setQ(double qn)
// {
//     Q = qn;
// }