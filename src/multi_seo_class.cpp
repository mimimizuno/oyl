#include "multi_seo_class.hpp"
//------ コンストラクタ（パラメータの初期設定）---------//
// 初期値無し
MultiSEO::MultiSEO() : R(0), Rj(0), Cj(0), C(0), Vd(0), Q(0), Vn(0), legs(0), V_sum(0), multi_num(1), tunnel_num(0)
{
    dE["up"] = 0.0;
    dE["down"] = 0.0;
    wt["up"] = 0.0;
    wt["down"] = 0.0;
}

// 初期値あり
MultiSEO::MultiSEO(double r, double rj, double cj, double c, double vd, int legscounts, int junction_num)
    : R(r), Rj(rj), Cj(cj), C(c), Vd(vd), Q(0.0), Vn(0.0), legs(legscounts),
      V_sum(0.0), multi_num(junction_num), tunnel_num(0)
{
    dE["up"] = 0.0;
    dE["down"] = 0.0;
    wt["up"] = 0.0;
    wt["down"] = 0.0;
}

//-----------セッター------------//
// パラメータセットアップ
void MultiSEO::setUp(double r, double rj, double cj, double c, double vd, int legscounts, int junction_num)
{
    R = r;
    Rj = rj;
    Cj = cj;
    C = c;
    Vd = vd;
    legs = legscounts;
    multi_num = junction_num;
}

// バイアス電圧を設定
void MultiSEO::setVias(const double vd)
{
    Vd = vd;
}

// V_sumを設定
void MultiSEO::setVsum(double v)
{
    V_sum = v;
}

// 接続情報を設定
void MultiSEO::setConnections(const std::vector<std::shared_ptr<BaseElement>> &conns)
{
    connections.clear();
    if (conns.size() > legs)
    {
        throw std::invalid_argument("The size of connections must match the number of legs.");
    }
    for (const auto &elem : conns)
    {
        if (this == elem.get())
        {
            throw std::invalid_argument("Cannot connect to itself.");
        }
        connections.push_back(elem);
    }
}

// 周囲の電圧を設定
void MultiSEO::setSurroundingVoltages()
{
    V_sum = 0;
    for (auto elem : connections)
    {
        V_sum += elem->getVn();
    }
}

// 振動子のパラメータ計算
void MultiSEO::setPcalc()
{
    Vn = (multi_num * (Cj * Q + C * Cj * V_sum) - Cj * tunnel_num * e) / (Cj * (legs * multi_num * C + Cj));
}

// 振動子のエネルギー計算
void MultiSEO::setdEcalc()
{
    dE["up"] = e * ((-(multi_num - 1) * legs + 2 * legs * tunnel_num) * C * e + Cj * (2 * Q - e) + 2 * C * Cj * V_sum) / (2 * Cj * (legs * multi_num * C + Cj));
    dE["down"] = -e * (-(-(multi_num - 1) * legs - 2 * legs * tunnel_num) * C * e + Cj * (2 * Q + e) + 2 * C * Cj * V_sum) / (2 * Cj * (legs * multi_num * C + Cj));
}

// 電荷の更新
void MultiSEO::setNodeCharge(const double dt)
{
    Q += (Vd - Vn) * dt / R;
}

// トンネル待ち時間計算(upまたはdownが正の時にwtを計算してtrueを返す)
bool MultiSEO::calculateTunnelWt()
{
    // 初期化
    wt["up"] = 0;
    wt["down"] = 0;
    if (dE["up"] > 0)
    {
        wt["up"] = 100;
        // ジャンクションの数からトンネルした回数を引いた分だけwtを計算して最小値を算出
        for (int rep = 0; rep < multi_num - abs(tunnel_num); rep++)
        {
            double tmpwt = (e * e * Rj / dE["up"]) * std::log(1 / Random());
            wt["up"] = std::min(tmpwt, wt["up"]);
        }
        return true;
    }
    if (dE["down"] > 0)
    {
        wt["down"] = 100;
        for (int rep = 0; rep < multi_num - abs(tunnel_num); rep++)
        {
            double tmpwt = (e * e * Rj / dE["down"]) * std::log(1 / Random());
            wt["down"] = std::min(tmpwt, wt["down"]);
        }
        return true;
    }
    return false;
}

// 振動子のトンネル
void MultiSEO::setTunnel(const std::string& direction)
{
    if (direction == "up")
    {
        // トンネル回数をカウント
        tunnel_num++;
        // トンネル回数がジャンクションの数が一致したときにQをアップデート、tunnel_numのリセット
        if(abs(tunnel_num) == multi_num)
        {
            Q -= e;
            tunnel_num = 0;
        }
    }
    else if (direction == "down")
    {
        tunnel_num--;
        if(abs(tunnel_num) == multi_num)
        {
            Q += e;
            tunnel_num = 0;
        }
    }
    else
    {
        throw std::invalid_argument("Invalid tunnel direction");
    }
}
//-----------ゲッター------------//

// ノード電圧を取得
double MultiSEO::getVn() const
{
    return Vn;
}

// // 接続されてる振動子を取得
// vector<shared_ptr<MultiSEO>> MultiSEO::getConnection() const
// {
//     return connection;
// }

// 接続されてる振動子の電圧の総和を取得
double MultiSEO::getSurroundingVsum() const
{
    return V_sum;
}

// dEの取得
std::map<std::string, double> MultiSEO::getdE() const
{
    return dE;
}

// Qの取得
double MultiSEO::getQ() const
{
    return Q;
}

// wtの取得
std::map<std::string, double> MultiSEO::getWT() const
{
    return wt;
}

//-------- 汎用処理 -------------//
// 0から1の間の乱数を生成
double MultiSEO::Random()
{
    static std::random_device rd;
    static std::mt19937 mt(rd());
    static std::uniform_real_distribution<double> dist(0.0, 1.0);
    return dist(mt);
}

//-------- テスト用 -----------//
// テスト用Rゲッター
double MultiSEO::getR() const
{
    return R;
}

// テスト用Rjゲッター
double MultiSEO::getRj() const
{
    return Rj;
}

// テスト用Cjゲッター
double MultiSEO::getCj() const
{
    return Cj;
}

// テスト用Cゲッター
double MultiSEO::getC() const
{
    return C;
}

// テスト用Vdゲッター
double MultiSEO::getVd() const
{
    return Vd;
}

// テスト用legsゲッター
int MultiSEO::getlegs() const
{
    return legs;
}

// テスト用dEセッター
void MultiSEO::setdE(const std::string &direction, double value)
{
    dE[direction] = value;
}

// テスト用Vnセッター
void MultiSEO::setVn(double vn)
{
    Vn = vn;
}

// テスト用Qnセッター
void MultiSEO::setQ(double qn)
{
    Q = qn;
}