#include "seo_class.hpp"
//------ コンストラクタ（パラメータの初期設定）---------//
// 初期値無し
SEO::SEO() : R(0), Rj(0), Cj(0), C(0), Vd(0), Q(0), Vn(0), legs(0), V_sum(0)
{
    dE["up"] = 0.0;
    dE["down"] = 0.0;
    wt["up"] = 0.0;
    wt["down"] = 0.0;
}

// 初期値あり
SEO::SEO(double r, double rj, double cj, double c, double vd, int legscounts)
    : R(r), Rj(rj), Cj(cj), C(c), Vd(vd), Q(0.0), Vn(0.0), legs(legscounts),
      V_sum(0.0)
{
    dE["up"] = 0.0;
    dE["down"] = 0.0;
    wt["up"] = 0.0;
    wt["down"] = 0.0;
}

//-----------セッター------------//
// パラメータセットアップ
void SEO::setUp(double r, double rj, double cj, double c, double vd, int legscounts)
{
    R = r;
    Rj = rj;
    Cj = cj;
    C = c;
    Vd = vd;
    legs = legscounts;
}

// バイアス電圧を設定
void SEO::setVias(const double vd)
{
    Vd = vd;
}

// V_sumを設定
void SEO::setVsum(double v)
{
    V_sum = v;
}

// 接続情報を設定
void SEO::setConnections(const std::vector<std::shared_ptr<BaseElement>>& conns) {
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
void SEO::setSurroundingVoltages()
{
    V_sum = 0;
    for (auto elem : connections)
    {
        V_sum += elem->getVn();
    }
}

// 振動子のパラメータ計算
void SEO::setPcalc()
{
    Vn = Q / Cj + (C / (Cj * (legs * C + Cj))) * (Cj * V_sum - legs * Q);
}

// 振動子のエネルギー計算
void SEO::setdEcalc()
{
    dE["up"] = -e * (e - 2 * (Q + C * V_sum)) / (2 * (legs * C + Cj));
    dE["down"] = -e * (e + 2 * (Q + C * V_sum)) / (2 * (legs * C + Cj));
}

// 電荷の更新
void SEO::setNodeCharge(const double dt)
{
    Q += (Vd - Vn) * dt / R;
}

// トンネル待ち時間計算(upまたはdownが正の時にwtを計算してtrueを返す)
bool SEO::calculateTunnelWt()
{
    // 初期化
    wt["up"] = 0;
    wt["down"] = 0;
    if (dE["up"] > 0)
    {
        wt["up"] = (e * e * Rj / dE["up"]) * std::log(1 / Random());
        return true;
    }
    if (dE["down"] > 0)
    {
        wt["down"] = (e * e * Rj / dE["down"]) * std::log(1 / Random());
        return true;
    }
    return false;
}

// 振動子のトンネル
void SEO::setTunnel(const std::string& direction)
{
    if (direction == "up")
    {
        Q += -e;
    }
    else if (direction == "down")
    {
        Q += e;
    }
    else
    {
        throw std::invalid_argument("Invalid tunnel direction");
    }
}
//-----------ゲッター------------//

// ノード電圧を取得
double SEO::getVn() const
{
    return Vn;
}

// 接続されてる振動子の電圧の総和を取得
double SEO::getSurroundingVsum() const
{
    return V_sum;
}

// dEの取得
std::map<std::string, double> SEO::getdE() const
{
    return dE;
}

// Qの取得
double SEO::getQ() const
{
    return Q;
}

// wtの取得
std::map<std::string, double> SEO::getWT() const
{
    return wt;
}

// oneway用の関数のため呼び出すとエラー発生
std::shared_ptr<BaseElement> SEO::getInternalElement(int index) const 
{
    throw std::runtime_error("This element does not have internal elements.");
}

//-------- 汎用処理 -------------//
// 0から1の間の乱数を生成
double SEO::Random()
{
    static std::random_device rd;
    static std::mt19937 mt(rd());
    static std::uniform_real_distribution<double> dist(0.0, 1.0);
    return dist(mt);
}

//-------- テスト用 -----------//
// テスト用Rゲッター
double SEO::getR() const
{
    return R;
}

// テスト用Rjゲッター
double SEO::getRj() const
{
    return Rj;
}

// テスト用Cjゲッター
double SEO::getCj() const
{
    return Cj;
}

// テスト用Cゲッター
double SEO::getC() const
{
    return C;
}

// テスト用Vdゲッター
double SEO::getVd() const
{
    return Vd;
}

// テスト用legsゲッター
int SEO::getlegs() const
{
    return legs;
}

// テスト用dEセッター
void SEO::setdE(const std::string &direction, double value)
{
    dE[direction] = value;
}

// テスト用Vnセッター
void SEO::setVn(double vn)
{
    Vn = vn;
}

// テスト用Qnセッター
void SEO::setQ(double qn)
{
    Q = qn;
}