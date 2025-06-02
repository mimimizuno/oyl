#include <iostream>
#include <cmath>
#include <fstream>
#include "memory_class.hpp"
#include "constants.hpp"
using namespace std;

// 多重のパラメータ計算
// int main(){
//     int N;
//     int leg;
//     double C;
//     double Cj_first;
//     double Cj_second;
//     double Vth;

//     cout << "input N leg C Vth" << endl;
//     cin >> N >> leg >> C >> Vth;

//     double formula_item_a = 2 * Vth;
//     double formula_item_b = 2 * Vth * leg * N * C - N * e;
//     double formula_item_c = - N * e * leg * (N - 1) * C;
//     double contens_in_root = pow(formula_item_b, 2.0) - 4 * formula_item_a * formula_item_c;
//     cout << "a = " << formula_item_a << " b = " << formula_item_b << " c = " << formula_item_c << " root = " << contens_in_root << endl; 

//     Cj_first = (-formula_item_b + pow(contens_in_root, 0.5)) / (2 * formula_item_a);
//     Cj_second = (-formula_item_b - pow(contens_in_root, 0.5)) / (2 * formula_item_a);
    
//     cout << "Vth = " << Vth << " leg = " << leg << " N = " << N << endl;
//     cout << "Cj_first = " << Cj_first << " Cj_second = " << Cj_second << endl;
// }

// メモリのヒステリシス
int main(){
    int leg;
    double C, CL, Cj, Rj, Vd = 0, V_surrounding;
    int tunnel_count = 0;
    double ans_right = 0, ans_left = 0, tmp1 = 0, tmp2 = 0;
    Rj = 0.001;
    string fileName = "../output/hysteresis.txt";
    cout << "input"<< endl << "leg C CL Cj V_surrounding" << endl;
    cin >> leg >> C >> CL >> Cj >> V_surrounding;
    ofstream ofs(fileName);

    auto memory = std::make_shared<MEMORY>();
    memory->setUp(Rj,Cj,C,CL,Vd,leg);
    memory->setVsum(V_surrounding);
    while(Vd < 0.1 && tunnel_count < 2){
        memory->setVias(Vd);
        memory->setPcalc();
        memory->setdEcalc();
        if(memory->getdE()["up1"] > 0){
            memory->setTunnel("up1");
            tunnel_count++;
            cout << "up1 " << Vd << endl;
        }
        else if(memory->getdE()["up2"] > 0){
            memory->setTunnel("up2");
            tunnel_count++;
            cout << "up2 " << Vd << endl;
        }
        else if(memory->getdE()["down1"] > 0){
            memory->setTunnel("down1");
            tunnel_count++;
            ans_right += Vd;
            tmp1 = Vd;
            cout << "down1 " << Vd << endl;
        }
        else if(memory->getdE()["down2"] > 0){
            memory->setTunnel("down2");
            tunnel_count++;
            cout << "down2 " << Vd << endl;
        }
        ofs << Vd << " " << memory->getVn() << endl;
        Vd += 0.0001;
    }
    tunnel_count = 0;

    while(Vd > -0.1 && tunnel_count < 4){
        memory->setVias(Vd);
        memory->setPcalc();
        memory->setdEcalc();
        if(memory->getdE()["up1"] > 0){
            memory->setTunnel("up1");
            tunnel_count++;
            if(tunnel_count < 2){ ans_right += Vd; tmp2 = Vd;};
            if(tunnel_count > 1) ans_left += Vd;
            cout << "up1 " << Vd << endl;
        }
        else if(memory->getdE()["up2"] > 0){
            memory->setTunnel("up2");
            tunnel_count++;
            cout << "up2 " << Vd << endl;
        }
        else if(memory->getdE()["down1"] > 0){
            memory->setTunnel("down1");
            tunnel_count++;
            cout << "down1 " << Vd << endl;
        }
        else if(memory->getdE()["down2"] > 0){
            memory->setTunnel("down2");
            tunnel_count++;
            cout << "down2 " << Vd << endl;
        }
        ofs << Vd << " " << memory->getVn() << endl;
        Vd -= 0.0001;
    }
    tunnel_count = 0;

    while(Vd <= 0 && tunnel_count < 4){
        memory->setVias(Vd);
        memory->setPcalc();
        memory->setdEcalc();
        if(memory->getdE()["up1"] > 0){
            memory->setTunnel("up1");
            tunnel_count++;
            cout << "up1 " << Vd << endl;
        }
        else if(memory->getdE()["up2"] > 0){
            memory->setTunnel("up2");
            tunnel_count++;
            cout << "up2 " << Vd << endl;
        }
        else if(memory->getdE()["down1"] > 0){
            memory->setTunnel("down1");
            tunnel_count++;
            ans_left += Vd;
            cout << "down1 " << Vd << endl;
        }
        else if(memory->getdE()["down2"] > 0){
            memory->setTunnel("down2");
            tunnel_count++;
            cout << "down2 " << Vd << endl;
        }
        ofs << Vd << " " << memory->getVn() << endl;
        Vd += 0.0001;
    }
    tunnel_count = 0;

    cout << "Vd+_center = " << ans_right / 2 << endl;
    cout << "Vd-_center = " << ans_left / 2 << endl;
    cout << "Vd_range = " << tmp1 - tmp2 << endl;
    string scriptFilename = fileName.substr(0, fileName.find_last_of('.')) + "_gnu.txt";
    std::ofstream gnuFile(scriptFilename);
    gnuFile << "#unset key\n";
    gnuFile << "#set title 'no'\n";
    gnuFile << "set terminal qt font \"Arial,10\"\n";
    gnuFile << "set xl 'Vd[V]'\n";
    gnuFile << "set yl 'Vn[V]'\n";
    gnuFile << "\n";
    gnuFile << "p '" << fileName << "' u 1:2 w steps lw 3";
}