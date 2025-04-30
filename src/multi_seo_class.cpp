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
      V_sum(0.0), connection(0), multi_num(junction_num), tunnel_num(0)
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
void MultiSEO::setConnections(const vector<shared_ptr<MultiSEO>> &connectedSEOs)
{
    connection.clear();
    if (connectedSEOs.size() > legs)
    {
        throw invalid_argument("The size of connections must match the number of legs.");
    }
    for (const auto &seo : connectedSEOs)
    {
        if (this == seo.get())
        {
            throw invalid_argument("Cannot connect to itself.");
        }
        connection.push_back(seo);
    }
}

// 周囲の電圧を設定
void MultiSEO::setSurroundingVoltages()
{
    V_sum = 0;
    for (auto seo : connection)
    {
        V_sum += seo->Vn;
    }
}

// 振動子のパラメータ計算
void MultiSEO::setPcalc()
{
    Vn = Q / Cj + (C / (Cj * (legs * C + Cj))) * (Cj * V_sum - legs * Q);
}

// 振動子のエネルギー計算
void MultiSEO::setdEcalc()
{
    dE["up"] = -e * (e - 2 * (Q + C * V_sum)) / (2 * (legs * C + Cj));
    dE["down"] = -e * (e + 2 * (Q + C * V_sum)) / (2 * (legs * C + Cj));
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
        wt["up"] = (e * e * Rj / dE["up"]) * log(1 / Random());
        return true;
    }
    if (dE["down"] > 0)
    {
        wt["down"] = (e * e * Rj / dE["down"]) * log(1 / Random());
        return true;
    }
    return false;
}

// 振動子のトンネル
void MultiSEO::setTunnel(const string direction)
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
        throw invalid_argument("Invalid tunnel direction");
    }
}
//-----------ゲッター------------//

// ノード電圧を取得
double MultiSEO::getVn() const
{
    return Vn;
}

// 接続されてる振動子を取得
vector<shared_ptr<MultiSEO>> MultiSEO::getConnection() const
{
    return connection;
}

// 接続されてる振動子の電圧の総和を取得
double MultiSEO::getSurroundingVsum() const
{
    return V_sum;
}

// dEの取得
map<string, double> MultiSEO::getdE() const
{
    return dE;
}

// Qの取得
double MultiSEO::getQ() const
{
    return Q;
}

// wtの取得
map<string, double> MultiSEO::getWT() const
{
    return wt;
}

//-------- 汎用処理 -------------//
// 0から1の間の乱数を生成
double MultiSEO::Random()
{
    static random_device rd;
    static mt19937 mt(rd());
    static uniform_real_distribution<double> dist(0.0, 1.0);
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
void MultiSEO::setdE(const string &direction, double value)
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