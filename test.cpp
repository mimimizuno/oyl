
typedef struct
{
    double Vd;
    double Vn;
    double V1;
    double V2;
    double V3;
    double V4;
    double V5;
    double V6;
    double Q;
    double dQ;
    double dE[2];
    double wt[3];
    int multi_num;
    int tunnel;
    int tunnel_num;
} multiseo;

// 多重振動子のパラメータ計算(&multiseo,足の本数,Cs,Cjs)
void multiseo_Pcalc(multiseo *p, int leg, double Cs, double Cjs);

// 多重振動子のエネルギー計算(&multiseo,足の本数,Cs,Cjs)
void multiseo_Ecalc(multiseo *p, int leg, double Cs, double Cjs);

// 多重振動子の電荷チャージ(&multiseo,R,dt)
double multiseo_charge(multiseo *p, multiseo *pmax, double R, double dt);

// 多重振動子の電荷チャージ(&multiseo,R,dt) *返り値なし
void multiseo_Charge(multiseo *p, multiseo *pmax, double R, double dt);

// 多重振動子の電荷チャージ(&multiseo,SEO_ROWS,SEO_COLUMNS,R,dt)(二次元配列) *返り値無し
void multiseo_2dimCharge(multiseo *p, int rows, int columns, double R, double dt);

// 多重振動子の電荷チャージ(&multiseo,SEO_PARTICLES,SEO_ROWS,SEO_COLUMNS,R,dt)(三次元配列) *返り値なし
void multiseo_3dimcharge(multiseo *p, int particles, int rows, int columns, double R, double dt);

// 多重振動子の電荷チャージ(&multiseo,SEO_PARTICLES,SEO_ROWS,SEO_COLUMNS,R,dt)(三次元配列)
double multiseo_3dimCharge(multiseo *p, int particles, int rows, int columns, double R, double dt);

// 多重振動子のwt計算(&multiseo,&seomax,Rj)
multiseo *multiseo_wt(multiseo *p, multiseo *pmax, double Rj);

// 多重振動子のwt計算(二次元配列)(&multiseo,SEO_ROWS,SEO_COLUMNS,Rj)
multiseo *multiseo_2dimwt(multiseo *p, int rows, int columns, double Rj);

// 多重振動子のwt計算(&multiseo,SEO_PARTICLES,SEO_ROWS,SEO_COLUMNS,Rj)(三次元配列)
multiseo *multiseo_3dimwt(multiseo *p, int particles, int rows, int columns, double Rj);

// 多重振動子のトンネル(&multiseo)
void multiseo_tunnel(multiseo *p);// 多重振動子のパラメータ計算(&multiseo,足の本数,Cs,Cjs)

void multiseo_Pcalc(multiseo *p, int leg, double Cs, double Cjs)
{
    double V_sum = p->V1 + p->V2 + p->V3 + p->V4 + p->V5 + p->V6;
    p->Vn = p->multi_num * (Cjs * p->Q + Cs * Cjs * V_sum - Cjs * p->tunnel_num * e) / Cjs * (leg * Cjs * (p->multi_num * Cs + Cjs));
}

// 多重振動子のエネルギー計算(&multiseo,足の本数,Cs,Cjs)
void multiseo_Ecalc(multiseo *p, int leg, double Cs, double Cjs)
{
    double V_sum = p->V1 + p->V2 + p->V3 + p->V4 + p->V5 + p->V6;
    // up
    p->dE[0] = e * ((-(p->multi_num - 1) * leg + 2 * leg * p->tunnel_num) * Cs * e + Cjs * (2 * p->Q - e) + 2 * Cs * Cjs * V_sum) / (2 * Cjs * (leg * p->multi_num * Cs + Cjs));
    // down
    p->dE[1] = - e * (-(-(p->multi_num - 1) * leg - 2 * leg * p->tunnel_num) * Cs * e + Cjs * (2 * p->Q + e) + 2 * Cs * Cjs * V_sum) / (2 * Cjs * (leg * p->multi_num * Cs + Cjs)); 
}

// 多重振動子の電荷チャージ(&multiseo,R,dt)
double multiseo_charge(multiseo *p, multiseo *pmax, double R, double dt)
{
    for (p; p <= pmax; p++)
    {
        p->dQ = (p->Vd - p->Vn) * dt / R;
        p->Q += p->dQ;
    }
    return 0.1;
}

// 多重振動子の電荷チャージ(&multiseo,R,dt) *返り値なし
void multiseo_Charge(multiseo *p, multiseo *pmax, double R, double dt)
{
    for (p; p <= pmax; p++)
    {
        p->dQ = (p->Vd - p->Vn) * dt / R;
        p->Q += p->dQ;
    }
}

// 多重振動子の電荷チャージ(&multiseo,SEO_ROWS,SEO_COLUMNS,R,dt)(二次元配列) *返り値無し
void multiseo_2dimCharge(multiseo *p, int rows, int columns, double R, double dt)
{
    for (int i = 0; i < rows * columns; i++)
    {
        p->dQ = (p->Vd - p->Vn) * dt / R;
        p->Q += p->dQ;
        p++;
    }
}

// 多重振動子の電荷チャージ(&multiseo,SEO_PARTICLES,SEO_ROWS,SEO_COLUMNS,R,dt)(三次元配列) *返り値なし
void multiseo_3dimcharge(multiseo *p, int particles, int rows, int columns, double R, double dt)
{
    for (int i = 0; i < particles * rows * columns; i++)
    {
        p->dQ = (p->Vd - p->Vn) * dt / R;
        p->Q += p->dQ;
        p++;
    }
}

// 多重振動子の電荷チャージ(&multiseo,SEO_PARTICLES,SEO_ROWS,SEO_COLUMNS,R,dt)(三次元配列)
double multiseo_3dimCharge(multiseo *p, int particles, int rows, int columns, double R, double dt)
{
    for (int i = 0; i < particles * rows * columns; i++)
    {
        p->dQ = (p->Vd - p->Vn) * dt / R;
        p->Q += p->dQ;
        p++;
    }
    return 0.1;
}

// 多重振動子のwt計算(&multiseo,&seomax,Rj)
multiseo *multiseo_wt(multiseo *p, multiseo *pmax, double Rj)
{
    int i = 0;
    double tmp = 100;
    multiseo *a = p;
    for (p; p <= pmax; p++)
    {
        for (i = 0; i < 3; i++)
        {
            if (i == 0)
            {
                p->wt[i] = 0.1;
            }
            else
            {
                if (p->dE[i - 1] > 0)
                {
                    for(int rep=0; rep < p->multi_num - abs(p->tunnel_num);rep++)
                    {
                        double mj_tunnele_wt = (e * e * Rj / p->dE[i - 1]) * log(1 / Random());
                        p->wt[i] = min(mj_tunnele_wt, p->wt[i]);
                    }
                }
                else
                {
                    p->wt[i] = 100;
                }
            }

            if (tmp > p->wt[i])
            {
                tmp = p->wt[i];
                p->tunnel = i;
                a = p;
            }
        }
    }
    return a;
}

// 多重振動子のwt計算(二次元配列)(&multiseo,SEO_ROWS,SEO_COLUMNS,Rj)
multiseo *multiseo_2dimwt(multiseo *p, int rows, int columns, double Rj)
{
    int i = 0;
    int j = 0;
    double tmp = 100;
    multiseo *a = p;
    for (i; i < rows * columns; i++)
    {
        for (j = 0; j < 3; j++)
        {
            if (j == 0)
            {
                p->wt[j] = 0.1;
            }
            else
            {
                if (p->dE[j - 1] > 0)
                {
                    for(int rep=0; rep < p->multi_num - abs(p->tunnel_num);rep++)
                    {
                        double mj_tunnele_wt = (e * e * Rj / p->dE[j - 1]) * log(1 / Random());
                        p->wt[j] = min(mj_tunnele_wt, p->wt[j]);
                    }
                }
                else
                {
                    p->wt[j] = 100;
                }
            }

            if (tmp > p->wt[j])
            {
                tmp = p->wt[j];
                p->tunnel = j;
                a = p;
            }
        }
        p++;
    }
    return a;
}
// 多重振動子のwt計算(&multiseo,SEO_PARTICLES,SEO_ROWS,SEO_COLUMNS,Rj)(三次元配列)
multiseo *multiseo_3dimwt(multiseo *p, int particles, int rows, int columns, double Rj)
{
    int i = 0;
    int j = 0;
    double tmp = 100;
    multiseo *a = p;
    for (i; i < particles * rows * columns; i++)
    {
        for (j = 0; j < 3; j++)
        {
            if (j == 0)
            {
                p->wt[j] = 0.1;
            }
            else
            {
                if (p->dE[j - 1] > 0)
                {
                    for(int rep=0; rep < p->multi_num - abs(p->tunnel_num);rep++)
                    {
                        double mj_tunnele_wt = (e * e * Rj / p->dE[j - 1]) * log(1 / Random());
                        p->wt[j] = min(mj_tunnele_wt, p->wt[j]);
                    }
                }
                else
                {
                    p->wt[j] = 100;
                }
            }

            if (tmp > p->wt[j])
            {
                tmp = p->wt[j];
                p->tunnel = j;
                a = p;
            }
        }
        p++;
    }
    return a;
}

// 多重振動子のトンネル(&multiseo)
void multiseo_tunnel(multiseo *p)
{
    if (p->tunnel == 1) // up direction
    {
        p->tunnel_num++;
        if(abs(p->tunnel_num) == p->multi_num)
        {
            p->Q -= e;
            p->tunnel_num = 0;
        }
    }
    else if (p->tunnel == 2) // down direction
    {
        p->tunnel_num--;
        if(abs(p->tunnel_num) == p->multi_num)
        {
            p->Q += e;
            p->tunnel_num = 0;
        }
    }
}