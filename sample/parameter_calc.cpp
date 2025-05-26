#include <iostream>
#include <cmath>

using namespace std;
constexpr double e = 0.1602;     // 電荷量

int main(){
    int N;
    int leg;
    double C;
    double Cj_first;
    double Cj_second;
    double Vth;

    cout << "N leg C Vth" << endl;
    cin >> N >> leg >> C >> Vth;

    double formula_item_a = 2 * Vth;
    double formula_item_b = 2 * Vth * leg * N * C - N * e;
    double formula_item_c = - N * e * leg * (N - 1) * C;
    double contens_in_root = pow(formula_item_b, 2.0) - 4 * formula_item_a * formula_item_c;
    cout << "a = " << formula_item_a << " b = " << formula_item_b << " c = " << formula_item_c << " root = " << contens_in_root << endl; 

    Cj_first = (-formula_item_b + pow(contens_in_root, 0.5)) / (2 * formula_item_a);
    Cj_second = (-formula_item_b - pow(contens_in_root, 0.5)) / (2 * formula_item_a);
    
    cout << "Vth = " << Vth << " leg = " << leg << " N = " << N << endl;
    cout << "Cj_first = " << Cj_first << " Cj_second = " << Cj_second << endl;
}