#include <gtest/gtest.h>
#include <stdexcept>
#include "seo_class.hpp"

// // GoogleTestのSetUpをオーバーライドしてパラメータのセットアップ
// class SEOTest : public ::testing::Test
// {
// protected:
//     unique_ptr<vector<vector<vector<shared_ptr<SEO>>>>> seoGrid;

//     void SetUp() override
//     {
//         int x = 2, y = 2, z = 2;

//         // 3次元ベクトル (サイズ: 2x2x2) を動的に生成
//         seoGrid = make_unique<vector<vector<vector<shared_ptr<SEO>>>>>(
//             z, vector<vector<shared_ptr<SEO>>>(y, vector<shared_ptr<SEO>>(x)));

//         // 各要素を生成して格納
//         for (int i = 0; i < z; ++i)
//         {
//             for (int j = 0; j < y; ++j)
//             {
//                 for (int k = 0; k < x; ++k)
//                 {
//                     (*seoGrid).at(i).at(j).at(k) = make_shared<SEO>(1.0, 0.001, 18.0, 2.0, 0.007, 3);
//                 }
//             }
//         }

//         // [0][0][0]の振動子に[1][0][0],[0][1][0],[0][0][1]を接続
//         vector<shared_ptr<SEO>> connections = {
//             (*seoGrid).at(1).at(0).at(0),
//             (*seoGrid).at(0).at(1).at(0),
//             (*seoGrid).at(0).at(0).at(1)};
//         (*seoGrid).at(0).at(0).at(0)->setConnections(connections);
//     }
// };

// // テストケース: コンストラクタの動作確認
// TEST_F(SEOTest, Constructor)
// {
//     // パラメータの検証
//     EXPECT_EQ((*seoGrid).at(0).at(0).at(0)->getR(), 1.0);
//     EXPECT_EQ((*seoGrid).at(0).at(0).at(1)->getRj(), 0.001);
//     EXPECT_EQ((*seoGrid).at(0).at(1).at(0)->getCj(), 18.0);
//     EXPECT_EQ((*seoGrid).at(1).at(0).at(0)->getC(), 2);
//     EXPECT_EQ((*seoGrid).at(0).at(1).at(1)->getVd(), 0.007);
//     EXPECT_EQ((*seoGrid).at(1).at(1).at(1)->getlegs(), 3);
// }

// // テストケース: 接続されてる振動子のポインタの追加確認
// TEST_F(SEOTest, AddConnections)
// {
//     // 接続されたポインタが正しいか確認
//     auto connections = (*seoGrid).at(0).at(0).at(0)->getConnection();
//     EXPECT_EQ(connections.size(), 3);
//     // 接続0: [1][0][0]
//     EXPECT_EQ(connections[0], (*seoGrid).at(1).at(0).at(0));
//     // 接続1: [0][1][0]
//     EXPECT_EQ(connections[1], (*seoGrid).at(0).at(1).at(0));
//     // 接続2: [0][0][1]
//     EXPECT_EQ(connections[2], (*seoGrid).at(0).at(0).at(1));

//     // 新しい接続のベクトルを作成
//     vector<shared_ptr<SEO>> tooManyConnections = {
//         (*seoGrid).at(1).at(0).at(0),
//         (*seoGrid).at(0).at(1).at(0),
//         (*seoGrid).at(0).at(0).at(1),
//         (*seoGrid).at(1).at(1).at(1)}; // 4つ目の接続で超過

//     // 例外がスローされるか確認
//     EXPECT_THROW((*seoGrid).at(0).at(0).at(0)->setConnections(tooManyConnections), invalid_argument);

//     // 自分自身への接続
//     vector<shared_ptr<SEO>> selfConnections = {
//         (*seoGrid).at(0).at(0).at(0),
//         (*seoGrid).at(1).at(0).at(0),
//         (*seoGrid).at(0).at(1).at(0)};
//     // 例外がスローされるか確認
//     EXPECT_THROW((*seoGrid).at(0).at(0).at(0)->setConnections(selfConnections), invalid_argument);
// }

// // テストケース: 接続されてる振動子のノード電圧を参照する動作確認
// TEST_F(SEOTest, SetSurroundingVoltages) {
//     // 接続のノード電圧を設定
//     (*seoGrid).at(1).at(0).at(0)->setVn(1);
//     (*seoGrid).at(0).at(1).at(0)->setVn(2);
//     (*seoGrid).at(0).at(0).at(1)->setVn(3);
//     // 周囲の電圧を設定
//     (*seoGrid).at(0).at(0).at(0)->setSurroundingVoltages();

//     // Vが正しく更新されているか確認
//     const auto& surroundingVoltage = (*seoGrid).at(0).at(0).at(0)->getSurroundingVsum();
//     // EXPECT_EQ(surroundingVoltages.size(), 3); // 接続数が一致しているか
//     // EXPECT_DOUBLE_EQ(surroundingVoltages.at(0), 1); // [1][0][0] のノード電圧
//     // EXPECT_DOUBLE_EQ(surroundingVoltages.at(1), 2); // [0][1][0] のノード電圧
//     // EXPECT_DOUBLE_EQ(surroundingVoltages.at(2), 3); // [0][0][1] のノード電圧
//     EXPECT_EQ(surroundingVoltage, 1+2+3);
// }

// // テストケース: パラメータ計算の動作確認
// TEST_F(SEOTest, SetPcalc) {
//     // 振動子を選択(r=1,rj=0.001,cj=18,c=2,vd=0.007,legs=3)
//     auto oscillator = (*seoGrid).at(0).at(0).at(0);

//     // 初期パラメータ設定
//     oscillator->setQ(2);   // 電荷量

//     // 接続のノード電圧を設定
//     (*seoGrid).at(1).at(0).at(0)->setVn(1);
//     (*seoGrid).at(0).at(1).at(0)->setVn(2);
//     (*seoGrid).at(0).at(0).at(1)->setVn(3);
//     // 周囲の電圧を設定
//     (*seoGrid).at(0).at(0).at(0)->setSurroundingVoltages();

//     // パラメータ計算
//     oscillator->setPcalc();

//     // ノード電圧計算: Vn = Q / Cj + (C / (Cj * (legs * C + Cj))) * (Cj * V_sum - legs * Q)
//     double expectedVn = 2.0 / 18.0 + (2.0 / (18.0 * (3 * 2.0 + 18.0))) * (18.0 * 6.0 - 3 * 2.0);
//     EXPECT_DOUBLE_EQ(oscillator->getVn(), expectedVn);

//     // 旧式の方法での計算を確認
//     // q1 = (C * (-Q + Cj * V1 + C * (2 * V1 - V2 - V3))) / (3 * C + Cj);
//     double q1 = (2.0 * (-2.0 + 18.0 * 1.0 + 2.0 * (2 * 1.0 - 2.0 - 3.0))) / (3 * 2.0 + 18.0);
//     // q2 = -((C * (Q - Cj * V2 + C * (V1 - 2 * V2 + V3))) / (3 * C + Cj));
//     double q2 = -((2.0 * (2.0 - 18.0 * 2.0 + 2.0 * (1.0 - 2 * 2.0 + 3.0))) / (3 * 2.0 + 18.0));
//     // q3 = -((C * (Q + C * (V1 + V2 - 2 * V3) - Cj * V3)) / (3 * C + Cj));
//     double q3 = -((2.0 * (2.0 + 2.0 * (1.0 + 2.0 - 2 * 3.0) - 18.0 * 3.0)) / (3 * 2.0 + 18.0));
//     // calcVn = (Q + q1 + q2 + q3) / Cj;
//     double calcVn = (2.0 + q1 + q2 + q3) / 18.0;
//     EXPECT_DOUBLE_EQ(oscillator->getVn(), calcVn);
// }

// // テストケース: エネルギー変化計算の動作確認
// TEST_F(SEOTest, SetdEcalc) {
//     // 振動子を選択(r=1,rj=0.001,cj=18,c=2,vd=0.007,legs=3)
//     auto oscillator = (*seoGrid).at(0).at(0).at(0);

//     // 初期パラメータ設定
//     oscillator->setQ(2);   // 電荷量

//     // 接続のノード電圧を設定
//     (*seoGrid).at(1).at(0).at(0)->setVn(1);
//     (*seoGrid).at(0).at(1).at(0)->setVn(2);
//     (*seoGrid).at(0).at(0).at(1)->setVn(3);
//     // 周囲の電圧を設定
//     (*seoGrid).at(0).at(0).at(0)->setSurroundingVoltages();

//     // パラメータ計算
//     oscillator->setdEcalc();

//     // エネルギー変化計算:
//     // dE["up"] = -e * (e - 2 * (Q + C * V_sum)) / (2 * (legs * C + Cj));
//     // dE["down"] = -e * (e + 2 * (Q + C * V_sum)) / (2 * (legs * C + Cj));
//     double expected_dE_up = -e * (e - 2 * (2.0 + 2.0 * 6.0)) / (2 * (3 * 2.0 + 18.0));
//     double expected_dE_down = -e * (e + 2 * (2.0 + 2.0 * 6.0)) / (2 * (3 * 2.0 + 18.0));
//     EXPECT_DOUBLE_EQ(oscillator->getdE()["up"], expected_dE_up);
//     EXPECT_DOUBLE_EQ(oscillator->getdE()["down"], expected_dE_down);
// }

// // テストケース: 電荷チャージの動作確認
// TEST_F(SEOTest, SetNodeCharge) {
//     // 振動子を選択(r=1,rj=0.001,cj=18,c=2,vd=0.007,legs=3)
//     auto oscillator = (*seoGrid).at(0).at(0).at(0);

//     // 初期パラメータ設定
//     oscillator->setQ(0);   // 初期電荷
//     oscillator->setVn(3.0); // ノード電圧
//     double dt = 0.1;        // 時間ステップ

//     // 電荷を更新: Q += (Vd - Vn) * dt / R;

//     oscillator->setNodeCharge(dt);

//     // 期待値を計算
//     double expectedQ = 0.0 + (0.007 - 3.0) * dt / 1.0;

//     // 更新後のQを確認
//     EXPECT_DOUBLE_EQ(oscillator->getQ(), expectedQ);

//     // さらに更新して確認
//     oscillator->setNodeCharge(dt);
//     expectedQ += (0.007 - 3.0) * dt / 1.0; // 2回目の更新
//     EXPECT_DOUBLE_EQ(oscillator->getQ(), expectedQ);
// }

// // テストケース: calculateTunnelWt の動作確認
// TEST_F(SEOTest, CalculateTunnelWt)
// {
//     // 振動子を選択(r=1,rj=0.001,cj=18,c=2,vd=0.007,legs=3)
//     auto oscillator = (*seoGrid).at(0).at(0).at(0);

//     // エネルギー変化量をセット
//     oscillator->setdE("up", 1);    // 上方向のエネルギー変化量
//     oscillator->setdE("down", -1); // 下方向のエネルギー変化量

//     // トンネル待ち時間を計算
//     oscillator->calculateTunnelWt();
//     // 結果を検証
//     EXPECT_NE(oscillator->getWT()["up"], 0);
//     EXPECT_EQ(oscillator->getWT()["down"], 0);

//     // エネルギー変化量をセット
//     oscillator->setdE("up", -1);  // 上方向のエネルギー変化量
//     oscillator->setdE("down", 1); // 下方向のエネルギー変化量

//     // トンネル待ち時間を計算
//     oscillator->calculateTunnelWt();
//     // 結果を検証
//     EXPECT_EQ(oscillator->getWT()["up"], 0);
//     EXPECT_NE(oscillator->getWT()["down"], 0);
// }

// // テストケース: トンネル発生の動作確認
// TEST_F(SEOTest, SetTunnel) 
// {
//     // 振動子を選択
//     auto oscillator = (*seoGrid).at(0).at(0).at(0);

//     // 初期電荷を設定
//     oscillator->setQ(1.0);    // 初期電荷量

//     // トンネル方向が "up" の場合
//     oscillator->setTunnel("up");
//     double expectedQ_up = 1.0 - e; // 電荷が -e 減少
//     EXPECT_DOUBLE_EQ(oscillator->getQ(), expectedQ_up);

//     // トンネル方向が "down" の場合
//     oscillator->setTunnel("down");
//     double expectedQ_down = expectedQ_up + e; // 電荷が +e 増加
//     EXPECT_DOUBLE_EQ(oscillator->getQ(), expectedQ_down);

//     // 無効なトンネル方向（例: 空文字列）では例外処理が発生することを確認
//     EXPECT_THROW(oscillator->setTunnel(""), invalid_argument);
    
// }