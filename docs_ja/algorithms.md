# 経路追従アルゴリズム詳解

本ドキュメントでは、リポジトリに実装された各アルゴリズムについて、制御則の
**導出**、**閉ループ解析**、**記号と実装（設定フィールド）の対応**、および
理論値とベンチマーク実測値の**突き合わせ**をまとめます。

- [0. 共通の枠組み](#0-共通の枠組み)
- [1. Pure Pursuit](#1-pure-pursuit)
- [2. Stanley](#2-stanley)
- [3. MPC（LTV / Riccati）](#3-mpcltv--riccati)
- [4. MPPI](#4-mppi)
- [5. ACC の縦方向制御則](#5-acc-の縦方向制御則)
- [6. 総合比較](#6-総合比較)
- [7. 参考文献](#7-参考文献)

関連ドキュメント: [`comparison_report.md`](comparison_report.md)（結果と図）、
[`../README.ja.md`](../README.ja.md)（ビルドと実行）。

---

## 0. 共通の枠組み

4 つのコントローラは同一のプラント・同一の経路表現・同一の縦方向制御則を共有
します。したがって結果の差はすべて**操舵則の違い**に由来します。

### 0.1 車両モデル

**後輪車軸**を基準点とする運動学的自転車モデル
（[`vehicle.hpp`](../include/path_tracking/vehicle.hpp)）:

$$
\dot{x} = v\cos\psi,\qquad
\dot{y} = v\sin\psi,\qquad
\dot{\psi} = \frac{v}{L}\tan\delta,\qquad
\dot{v} = a
$$

ここで $L = 2.7\,\mathrm{m}$ はホイールベース、$\psi$ は方位角、$\delta$ は
前輪操舵角（正 = 左）、$a$ は前後加速度です。シミュレータは
$\Delta t = 0.01\,\mathrm{s}$（100 Hz）の陽的 Euler 法で、$v$ → 位置 → $\psi$
の順に更新します:

$$
v_{k+1} = \max(0,\; v_k + a_k\Delta t),\quad
x_{k+1} = x_k + v_{k+1}\cos\psi_k\,\Delta t,\quad
\psi_{k+1} = \psi_k + \frac{v_{k+1}\tan\delta_k}{L}\Delta t
$$

質量・タイヤ・アクチュエータ遅れは扱いません。本試験の速度域
（$\le 8\,\mathrm{m/s}$、横加速度 $\le 1.5\,\mathrm{m/s^2}$）では運動学モデルで
十分であり、比較の焦点を*アルゴリズム*に保てます。

曲率 $\kappa$ を維持するのに必要な定常操舵角は $\dot\psi = v\kappa$ から直ちに
得られます:

$$
\boxed{\;\delta_{\mathrm{ss}} = \arctan(L\kappa)\;}
$$

この恒等式が MPC の曲率フィードフォワード（§3.5）であり、Pure Pursuit が円弧
上で厳密になる理由（§1.4）でもあります。

### 0.2 経路と誤差の定義

経路は $p_i = (x_i, y_i, \psi_i, \kappa_i, s_i, v_{\mathrm{ref},i})$ を
$\Delta s \approx 0.54\,\mathrm{m}$ 間隔で並べた折れ線です
（[`path_io.hpp`](../include/path_tracking/path_io.hpp)）。各コントローラは
点 $(x, y)$ に対する最近傍インデックス

$$
i^\* = \arg\min_{i \in [\,\hat\imath - W,\; \hat\imath + W\,]}\; (x_i - x)^2 + (y_i - y)^2
$$

を、前回インデックス $\hat\imath$ 周りの窓 $W$（コントローラでは $W = 300$、
MPPI のロールアウト内では $200$）で探索します。**横方向誤差**は左法線
$\hat n = (-\sin\psi_{i^\*}, \cos\psi_{i^\*})$ への射影として定義されます:

$$
e = -\sin\psi_{i^\*}\,(x - x_{i^\*}) + \cos\psi_{i^\*}\,(y - y_{i^\*})
\qquad (e > 0 \Rightarrow \text{車両が経路の左側})
$$

**方位誤差**はコード中に 2 つの符号規約が存在します。以下では各コントローラが
実際に使う定義をそのまま用います:

$$
\theta_e = \psi_{i^\*} - \psi \quad (\text{Stanley、指標計算}),
\qquad
\psi_e = \psi - \psi_{i^\*} \quad (\text{MPC}) .
$$

基準経路は*離散サンプル*であるため、$\psi_{i^\*}$ と $\kappa_{i^\*}$ は区分定数
であり、最近傍インデックスが 1 つ進むたびに

$$
\Delta\psi_{i^\*} \approx \kappa\,\Delta s
$$

だけ跳びます（$R = 25\,\mathrm{m}$ のカーブでは
$0.0216\,\mathrm{rad} = 1.24^\circ$）。局所接線を直接使うコントローラ
（Stanley、MPC）はこの量子化を操舵指令に持ち込みますが、Pure Pursuit は
持ち込みません（§6.2）。

### 0.3 基準速度プロファイル

曲率制限をかけたのち、後退パスで平滑化してカーブの*手前*で減速させます
（`assign_speed_profile`）:

$$
v_{\mathrm{ref},i} = \mathrm{clamp}\!\left(\sqrt{\frac{a_{\mathrm{lat}}^{\max}}{|\kappa_i|}},\; v_{\min},\; v_{\max}\right),
\qquad
v_{\mathrm{ref},i-1} \leftarrow \min\!\left(v_{\mathrm{ref},i-1},\; \sqrt{v_{\mathrm{ref},i}^2 + 2 a_{\mathrm{lat}}^{\max}\Delta s}\right)
$$

$a_{\mathrm{lat}}^{\max} = 1.5\,\mathrm{m/s^2}$、$v_{\min} = 1.5$、
$v_{\max} = 8.0\,\mathrm{m/s}$。本経路では直線で $8.0\,\mathrm{m/s}$、
$R = 30$ で $6.7$、$R = 25$ で $6.1\,\mathrm{m/s}$ となります。

### 0.4 縦方向制御則とアクチュエータ制限

4 手法で共通（[`controller.hpp`](../include/path_tracking/controller.hpp)）:

$$
a = \mathrm{clamp}\big(k_p\,(v_{\mathrm{ref},i^\*} - v),\; -a_{\mathrm{dec}}^{\max},\; a_{\mathrm{acc}}^{\max}\big),
\qquad k_p = 1.2\,\mathrm{s^{-1}},\; a_{\mathrm{acc}}^{\max} = 1.5,\; a_{\mathrm{dec}}^{\max} = 3.0
$$

| | Pure Pursuit | Stanley | MPC | MPPI |
|---|---|---|---|---|
| $\lvert\delta\rvert \le$ | 0.6 rad (34.4°) | 0.6 rad | 0.6 rad | 0.6 rad |
| $\lvert\dot\delta\rvert \le$ | —（なし） | 4 rad/s | 4 rad/s | 4 rad/s |

レート制限は 1 制御周期あたり $4 \times 0.01 = 0.04\,\mathrm{rad} = 2.29^\circ$
に相当します。Pure Pursuit だけはレート制限を必要としません。これ自体が 1 つの
結果です（§6.2）。

---

## 1. Pure Pursuit

[`pure_pursuit.hpp`](../include/path_tracking/pure_pursuit.hpp) — 幾何的手法、
後輪車軸基準（Coulter, 1992）。

### 1.1 制御則

弧長で $\ell_d$ 以上前方にある最初の経路点を目標とし、車両方位と目標点方向の
なす角 $\alpha$ を測り、車両に接して目標点を通る円弧に沿って操舵します:

$$
\alpha = \mathrm{wrap}\big(\operatorname{atan2}(y_{\mathrm{tgt}} - y,\; x_{\mathrm{tgt}} - x) - \psi\big),
\qquad
\delta = \arctan\!\left(\frac{2L\sin\alpha}{\ell_d}\right)
$$

先読み距離は速度に応じて可変です:

$$
\ell_d = \mathrm{clamp}(k_v\, v + \ell_{d,\min},\; \ell_{d,\min},\; \ell_{d,\max}),
\qquad k_v = 0.6\,\mathrm{s},\; \ell_{d,\min} = 3\,\mathrm{m},\; \ell_{d,\max} = 20\,\mathrm{m}
$$

### 1.2 導出

目標点を通る円弧の半径を $R_{pp}$ とします。三角形（車両・円弧中心・目標点）は
2 辺が $R_{pp}$、弦が $\ell_d$ で、弦と車両方位のなす角が $\alpha$、車両方位は
半径に直交するので、正弦定理より

$$
\frac{\ell_d}{\sin 2\alpha} = \frac{R_{pp}}{\sin\left(\tfrac{\pi}{2} - \alpha\right)}
\;\Longrightarrow\;
R_{pp} = \frac{\ell_d}{2\sin\alpha}
\;\Longrightarrow\;
\kappa_{\mathrm{cmd}} = \frac{2\sin\alpha}{\ell_d}
$$

これを $\delta = \arctan(L\kappa)$ に代入すると上の制御則になります。目標点の
車両座標系での横方向オフセット $y_{\mathrm{tgt}}^{\mathrm{veh}} = \ell_d \sin\alpha$
を使うと、指令曲率は

$$
\kappa_{\mathrm{cmd}} = \frac{2\,y_{\mathrm{tgt}}^{\mathrm{veh}}}{\ell_d^{\,2}}
$$

と書けます（§1.4〜1.5 で使用）。コードでは `atan2(2L sin α, ld_eff)` としており、
$\ell_{d,\mathrm{eff}}$ は離散点までの*実距離*（$\ge \ell_d$）です。

### 1.3 直線上の閉ループ特性

経路を $x$ 軸に取り、$\psi_e = \psi - \psi_p$ を用いて $e,\psi_e$ について線形化
します。目標点は経路上 $\ell_d$ 前方にあるので
$\alpha \approx -e/\ell_d - \psi_e$、$\delta \approx (2L/\ell_d)\alpha$ です。
$\dot e = v\sin\psi_e \approx v\psi_e$、$\dot\psi_e \approx (v/L)\delta$ より:

$$
\boxed{\;\ddot e + \frac{2v}{\ell_d}\dot e + \frac{2v^2}{\ell_d^{\,2}} e = 0\;}
$$

すなわち 2 次系であり、

$$
\omega_n = \frac{\sqrt2\,v}{\ell_d},
\qquad
\zeta = \frac{1}{\sqrt2} \approx 0.707
$$

ここから実測挙動を説明する 2 つの帰結が得られます:

- **減衰比が定数**であり、速度・先読み距離・ホイールベースに依存しません。
  Pure Pursuit は本質的によく減衰しており、レート制限なしでもベンチマークで
  最も滑らかな操舵（1.6 °/s RMS）になる理由がここにあります。
- **帯域は速度とともに飽和**します。$\ell_d = k_v v + \ell_{d,\min}$ なので
  $v$ が大きいと $\omega_n \to \sqrt2/k_v = 2.36\,\mathrm{rad/s}$ に漸近します。
  したがって高速ほど追従精度は落ち、$k_v$ が応答性（$\ell_d$ 小）と安定性・
  滑らかさ（$\ell_d$ 大）を決める唯一のつまみになります。
  $v = 8\,\mathrm{m/s}$ では $\ell_d = 7.8\,\mathrm{m}$、
  $\omega_n = 1.45\,\mathrm{rad/s}$、整定時間
  $\approx 3/(\zeta\omega_n) \approx 2.9\,\mathrm{s}$。

### 1.4 定曲率円弧では厳密

車両が曲率 $\kappa$ の円弧*上*にあるとき、目標点も同じ円上の弧長 $\ell_d$ 前方に
あり、その車両座標系での横方向オフセットは弦の幾何から厳密に
$y^{\mathrm{veh}} = \kappa \ell_d^{\,2}/2$ です。したがって

$$
\kappa_{\mathrm{cmd}} = \frac{2}{\ell_d^{\,2}}\cdot\frac{\kappa \ell_d^{\,2}}{2} = \kappa
$$

となり、$e = 0$ が $\ell_d$ の値によらず厳密な平衡点になります。すなわち
**Pure Pursuit は円弧上で定常偏差を持ちません**。実測トレースでも定曲率区間の
平均横方向誤差は $-0.002\,\mathrm{m}$（$R = 200$）、$-0.002$（$R = 150$）、
$-0.006$（$R = 30$）、$-0.012\,\mathrm{m}$（$R = 25$）と、ほぼゼロです。
よく引用される「$\kappa\ell_d^2/8$ のショートカット」は*折れ線*の弦を追う場合の
話であり、曲率連続な基準経路には当てはまりません。

### 1.5 クロソイド上の挙動 — 誤差が実際に出る場所

曲率が線形に増加する緩和曲線 $\kappa(s) = c\,s$（車両位置を原点とする）では、
目標点の横方向オフセットは
$y^{\mathrm{veh}} = \int_0^{\ell_d}\!\!\int_0^{u}\kappa\,\mathrm{d}\sigma\,\mathrm{d}u = c\,\ell_d^{\,3}/6$
となるので、

$$
\kappa_{\mathrm{cmd}} = \frac{2}{\ell_d^{\,2}}\cdot\frac{c\,\ell_d^{\,3}}{6} = \frac{c\,\ell_d}{3} = \kappa\!\left(s + \frac{\ell_d}{3}\right)
$$

すなわち **先読みは $\ell_d/3$（ここでは約 2.5 m）の曲率プレビューとして働きます**。
その結果、カーブが始まる*前*に切り始め（内側へ）、カーブが終わる*前*に戻し
始めます（外側へ）。実測トレースはまさにこの通りで、最急カーブでは進入
クロソイド $s = 914\,\mathrm{m}$ でピーク $-0.198\,\mathrm{m}$（内側）、脱出
クロソイド $s = 961\,\mathrm{m}$ で $+0.163\,\mathrm{m}$（外側）となり、その間の
定曲率区間は 0.01 m 以内で追従しています。本ベンチマークにおける Pure Pursuit の
最大誤差は*カーブ*の誤差ではなく*緩和区間*の誤差です。

### 1.6 パラメータ

| フィールド | 記号 | 既定値 | 効果 |
|---|---|---|---|
| `wheelbase` | $L$ | 2.7 m | 車両幾何。同一曲率に対する $\delta$ を決める |
| `ld_gain` | $k_v$ | 0.6 s | 単位速度あたりの先読み距離。高速帯域 $\sqrt2/k_v$ を決める |
| `ld_min` | $\ell_{d,\min}$ | 3.0 m | 低速でも目標点を前方に保つ |
| `ld_max` | $\ell_{d,\max}$ | 20.0 m | プレビュー上限。急カーブのショートカットを防ぐ |
| `steer_limit` | $\delta_{\max}$ | 0.6 rad | 飽和 |

**計算コスト:** 窓付き最近傍探索 1 回と前方走査のみ — 0.35 µs/周期で 4 手法中
最小。

---

## 2. Stanley

[`stanley.hpp`](../include/path_tracking/stanley.hpp) — 幾何的手法、
**前輪車軸**基準（2005 DARPA Grand Challenge 優勝車 Stanley）。

### 2.1 制御則

基準点は前輪車軸

$$
(x_f, y_f) = (x + L\cos\psi,\; y + L\sin\psi)
$$

であり、最近傍点・方位誤差 $\theta_e = \psi_{i^\*} - \psi$・横方向誤差 $e_f$ は
すべてこの点で評価されます:

$$
\boxed{\;\delta = k_\psi\,\theta_e - \arctan\!\left(\frac{k\,e_f}{k_{\mathrm{soft}} + v}\right)\;}
$$

マイナス符号は本リポジトリの規約によるものです（$e_f > 0$ は車両が経路の左側に
あることを意味し、これを補正するには右へ操舵する必要がある）。既定値は
$k = 2.5$、$k_{\mathrm{soft}} = 1.0\,\mathrm{m/s}$、$k_\psi = 1.0$。

### 2.2 収束性

前輪は方向 $\psi + \delta$ に速さ $v_f = v/\cos\delta \approx v$ で進むので、
横方向誤差は $\dot e_f = -v_f\sin(\theta_e - \delta)$ に従います。制御則
（$k_\psi = 1$）を代入すると $\theta_e$ が完全に相殺されます:

$$
\dot e_f = -v\sin\!\left(\arctan\frac{k e_f}{k_{\mathrm{soft}} + v}\right)
        = -\frac{v\,k\,e_f}{\sqrt{(k_{\mathrm{soft}} + v)^2 + (k e_f)^2}}
$$

2 つの領域に分かれます:

- **小偏差**（$k e_f \ll k_{\mathrm{soft}} + v$）:
  $\dot e_f \approx -\dfrac{v k}{k_{\mathrm{soft}} + v}\,e_f$ で指数収束。
  $v \gg k_{\mathrm{soft}}$ では時定数が $1/k = 0.4\,\mathrm{s}$ となり、
  Pure Pursuit と異なり**速度に依存しません**。高速域で Stanley が好まれた
  理由がこれです。
- **大偏差**: $\dot e_f \to -v\,\mathrm{sign}(e_f)$ — ほぼ全速度で経路へ
  接近します。$\arctan$ が飽和ゲインとして働き発散しません。
  $V = \tfrac12 e_f^2$ とすれば $e_f \ne 0$ で $\dot V < 0$ なので、本モデルでは
  大域収束します。

$k_{\mathrm{soft}}$ は $v \to 0$ でゲイン $k/v$ が発散して停止時にチャタリング
するのを防ぐための項です。

### 2.3 カーブでの定常偏差 — 厳密な結果

Stanley が零にするのは**前輪車軸**の誤差であり、定曲率カーブではそれが厳密に
成り立ちます。半径 $R$ の円では必要操舵角が $\tan\delta = L/R_r$（$R_r$ は
後輪車軸の旋回半径）であり、車両方位は後輪車軸の描く円に接し、前輪車軸投影点
での経路接線はそこから $\arctan(L/R_r)$ だけ回転しています。つまり方位項だけで
必要な操舵量がちょうど供給されます:

$$
\theta_e = \arctan\!\left(\frac{L}{R_r}\right) = \delta_{\mathrm{required}}
\;\Longrightarrow\;
\arctan\!\left(\frac{k e_f}{k_{\mathrm{soft}} + v}\right) = 0
\;\Longrightarrow\;
e_f = 0
$$

前輪車軸が経路円上に正確に乗ると、指標の測定点である後輪車軸は半径
$\sqrt{R^2 - L^2}$ の円上を通ることになり、カーブの**内側**へ

$$
\boxed{\;e_{\mathrm{rear}} = R - \sqrt{R^2 - L^2} \;\approx\; \frac{L^2\kappa}{2}\;}
$$

だけずれます。これは*幾何的*なオフセットであって、フィードフォワードの欠如が
原因ではありません（制御対象である前輪車軸誤差はすでに 0 なので、曲率
フィードフォワードを足しても解消しません）。実測との比較:

| カーブ | $\kappa$ [1/m] | 予測 $R - \sqrt{R^2 - L^2}$ [m] | 実測平均 CTE [m] | 比 |
|---|---|---|---|---|
| $R = 200$（左） | $+0.0050$ | $+0.0182$ | $+0.0169$ | 0.93 |
| $R = 150$（左） | $+0.0067$ | $+0.0243$ | $+0.0225$ | 0.93 |
| $R = 30$（右） | $-0.0333$ | $-0.1218$ | $-0.1155$ | 0.95 |
| $R = 25$（右） | $-0.0400$ | $-0.1462$ | $-0.1391$ | 0.95 |

残る 5〜7 % のずれは 100 Hz 離散時間ループの数値的アーティファクトです。同じ
制御則を理想円に対して単独で回すと実測値を再現し（$R = 200$ で 0.01691 m）、
$\Delta t \to 0$ で解析解へ収束します（$\Delta t = 1\,\mathrm{ms}$ で比 0.99）。

**帰結:** レポートの Stanley の `cte_max` 0.141 m は過渡誤差ではなく、
$R = 25$ カーブでのこの定常偏差そのものです。誤差は $L^2\kappa/2$ に比例する
ため、高速道路（$\kappa \to 0$）では無視でき、急旋回では支配的になります。
レポートで観測された傾向はまさにこの性質です。

### 2.4 位相進み（リード）フィルタ

急カーブでの振動を抑えるための 1 次差分項（任意）:

$$
\delta^{\mathrm{out}}_n = (1 + c)\,\delta_n - c\,\delta_{n-1},
\qquad
C(z) = (1 + c) - c z^{-1},
\qquad c = 0.15
$$

これは $\delta^{\mathrm{out}} \approx \delta + c\,\Delta t\,\dot\delta$ と等価で、
微分時間 1.5 ms の微分動作にあたります。なお `lead_state_` はフィルタ*前*の指令を
保持するため、フィルタは自身の出力ではなく生の制御則を微分します。
`lead_gain = 0` で無効化できます。定常状態（$\delta$ 一定）では寄与が 0 なので、
§2.3 の結論は変わりません。

### 2.5 パラメータ

| フィールド | 記号 | 既定値 | 効果 |
|---|---|---|---|
| `k_cross` | $k$ | 2.5 | 収束速度。高速域での時定数は $\approx 1/k$ |
| `k_soft` | $k_{\mathrm{soft}}$ | 1.0 m/s | 低速時のソフトニング（0 除算回避） |
| `k_heading` | $k_\psi$ | 1.0 | 方位誤差の重み。**1.0 であることが §2.3 を厳密にする** |
| `lead_gain` | $c$ | 0.15 | 急カーブでの位相進み |
| `steer_rate_limit` | $\dot\delta_{\max}$ | 4 rad/s | アクチュエーションの現実性 |

**計算コスト:** 窓付き最近傍探索 1 回 — 0.37 µs/周期。

---

## 3. MPC（LTV / Riccati）

[`mpc.hpp`](../include/path_tracking/mpc.hpp) — 横方向誤差ダイナミクスに対する
有限ホライズン線形 2 次制御を毎周期解き直します。

### 3.1 誤差状態と予測モデル

$$
z = \begin{bmatrix} e \\ \dot e \\ \psi_e \\ \dot\psi_e \end{bmatrix},
\qquad u = \delta,
\qquad
z_{k+1} = A z_k + B u_k
$$

毎周期、観測状態から初期化します（$\dot e = v\sin\psi_e$ は運動学モデルに対して
厳密、ヨーレート誤差は 0 にリセット）:

$$
z_0 = \begin{bmatrix} e & v\sin\psi_e & \psi_e & 0\end{bmatrix}^{\!\top},
\qquad \psi_e = \psi - \psi_{i^\*}
$$

$T = 0.05\,\mathrm{s}$（`pred_dt`）、速度をホライズン内で
$v = \max(v_{\mathrm{meas}}, 0.5)$ に固定すると:

$$
A = \begin{bmatrix}
1 & T & 0 & 0\\
0 & 1 & Tv & 0\\
0 & 0 & 1 & T\\
0 & 0 & 0 & 1
\end{bmatrix},
\qquad
B = \begin{bmatrix} 0 \\ 0 \\ 0 \\ \dfrac{v}{L} \end{bmatrix}
$$

すなわち $\ddot e = v\psi_e$、$\ddot\psi_e = (v/L)\,\delta$ です。明示しておく
べきモデル化上の選択が 2 つあります:

- 操舵は**ヨー角加速度**チャネルに入るため、$u$ は方位まで 2 回積分されます。
  結果として得られるフィードバックは $\psi_e$ に対する純比例ゲインではなく、
  進み遅れ（lead–lag）的な性格を持ちます。
- 曲率外乱項（$\dot\psi_e$ における $-v\kappa$）は**モデルに含まれていません**。
  ホライズン内では基準経路が局所的に直線であると仮定しています。曲率は §3.5 の
  フィードフォワードのみで扱われます。言い換えると、**この MPC は将来の経路を
  先読みしていません** — ホライズンはフィードバックゲインの形状を決めるだけで、
  使われる曲率情報は現在の最近傍点の $\kappa_{i^\*}$ だけです。

$A$、$B$ は $v$ に依存するため周期をまたげば線形時*変*ですが、1 つのホライズン
内では時不変です。

### 3.2 コストと解法

$$
J = \sum_{k=0}^{N-1}\left(z_k^\top Q z_k + R\,u_k^2\right) + z_N^\top Q_f z_N,
\qquad
Q = \mathrm{diag}(q_e, q_{\dot e}, q_\psi, q_{\dot\psi}),
\quad Q_f = \gamma Q
$$

これを後退 Riccati 漸化式で最小化します（制約なし問題の厳密解であり、QP
ソルバは不要です）:

$$
P_N = Q_f,\qquad
K_k = \frac{B^\top P_{k+1} A}{R + B^\top P_{k+1} B},\qquad
P_k = Q + A^\top P_{k+1} A - A^\top P_{k+1} B K_k
$$

用いるのは最初のゲインのみ（後退ホライズン）: $u_0 = -K_0 z_0$。既定値は
$q_e = 12$、$q_{\dot e} = 2$、$q_\psi = 8$、$q_{\dot\psi} = 2$、$R = 12$、
$\gamma = 4$、$N = 30$（ホライズン $1.5\,\mathrm{s}$）。

### 3.3 得られるゲイン

$v = 8\,\mathrm{m/s}$ で漸化式を評価すると:

$$
K_0 = \begin{bmatrix} 0.479 & 0.500 & 1.674 & 0.333\end{bmatrix}
$$

$A - BK_0$ の閉ループ固有値は $|\lambda| = 0.907, 0.907, 0.891, 0.319$ であり、
支配的時定数は $-T/\ln 0.907 = 0.52\,\mathrm{s}$ です。同じ速度での
Pure Pursuit の 0.97 s のおよそ 2 倍の速さであり、これが精度 5 倍差の主因の
1 つです。

$N = 30$ は無限ホライズン（DARE）解にすでに収束しています:

| $N$ | 5 | 10 | 20 | **30** | 60 | $\infty$ |
|---|---|---|---|---|---|---|
| $K_{0,1}$（$e$ へのゲイン） | 0.012 | 0.139 | 0.410 | **0.479** | 0.483 | 0.483 |

したがって本コントローラは数値的には「よく調整された LQR ＋ フィードフォワード」
であり、後退ホライズンが実際に買っているのは真の先読みではなく*現在速度での
再線形化*です。`horizon` を 20 程度より短くして初めて挙動が変わり（そして
悪化し）ます。

### 3.4 重みの効き方

$R/q_e$ が積極性を決めます。ループゲインはおよそ $\sqrt{q_e/R}$ に比例するので、
`r_steer` を 2 倍にすると指令はおよそ 30 % 滑らかになり、誤差も同程度広がります。
$q_{\dot e}$、$q_{\dot\psi}$ は速度項を直接罰することで減衰を与えます。
$\gamma = Q_f/Q$ は短ホライズンでのみ重要で、$N = 30$ では影響は 1 % 未満です。

### 3.5 曲率フィードフォワード

$$
\delta = \mathrm{clamp}\big(\underbrace{\arctan(L\kappa_{i^\*})}_{\text{フィードフォワード}} \;\underbrace{-\,K_0 z_0}_{\text{フィードバック}}\big),
\qquad
|\dot\delta| \le 4\,\mathrm{rad/s}
$$

フィードフォワードは運動学モデルに対して*厳密*であり（§0.1）、さらに MPC は
指標の測定点と同じ後輪車軸の誤差を制御対象にしているため、Stanley（§2.3）と
違って**幾何的な定常偏差を持ちません**。MPC が 0.008 m RMS に到達する主因は
先読みではなくこの 2 点です。

### 3.6 知っておくべき数値的アーティファクト

MPC の操舵レート RMS（46.6 °/s、4 手法中 2 番目に大きい）は LQR に由来する
ものではありません。$R = 25\,\mathrm{m}$ カーブでは指令が周期 9 の鋸歯状に
なります。最近傍インデックスは $\Delta s / v = 0.54/6.13 = 0.088\,\mathrm{s}$
$= 9$ 制御周期ごとに 1 つ進み、そのたびに基準方位が
$\kappa\Delta s = 0.0216\,\mathrm{rad}$ 跳びます。ゲインを通すとこれは

$$
\Delta\delta \approx (K_{0,3} + v\,K_{0,2})\,\kappa\,\Delta s \approx 0.12\,\mathrm{rad} \approx 7^\circ
$$

程度の指令ジャンプになり、4 rad/s のレート制限がそれを 2 周期に引き伸ばします
（実測の振れ幅は 3.7° → 8.3°、すなわち 4.6° p-p が 9 周期ごとに反復）。この鋸歯の
平均値は $\arctan(L\kappa) = 6.15^\circ$ をわずかに下回るため、同カーブに残る
$+0.028\,\mathrm{m}$（外側）のオフセットもこれで説明できます。対策としては
サンプル間で基準値を補間する、あるいは $\psi_{i^\*}$ をローパスする方法が
ありますが、本実装では行っていません。地図ベースのコントローラで実際によく
起きる失敗モードなので、あえて可視化したままにしてあります。

### 3.7 パラメータと計算コスト

| フィールド | 記号 | 既定値 | 効果 |
|---|---|---|---|
| `horizon` | $N$ | 30 | ステップ数。DARE 収束には 20 以上必要 |
| `pred_dt` | $T$ | 0.05 s | 予測刻み。$NT = 1.5\,\mathrm{s}$ |
| `q_cte` / `q_yaw` | $q_e$ / $q_\psi$ | 12 / 8 | 誤差の重み |
| `q_cte_rate` / `q_yaw_rate` | $q_{\dot e}$ / $q_{\dot\psi}$ | 2 / 2 | 減衰 |
| `r_steer` | $R$ | 12 | 操作量の重み。大きいほど滑らかで低精度 |
| `qf_scale` | $\gamma$ | 4 | 終端重み |

**計算コスト:** $n = 4$ に対する $O(N n^3)$ — 小さな密行列演算 30 回で
1.52 µs/周期。Eigen を使うのはここだけです。

---

## 4. MPPI

[`mppi.hpp`](../include/path_tracking/mppi.hpp) — Model Predictive Path
Integral（Williams et al., 2016/2017）: サンプリングに基づく後退ホライズン制御。

### 4.1 考え方

最適解を解く代わりに、$K$ 本のノイズ付き操舵系列をサンプリングし、非線形モデルで
それぞれ展開して、**コスト重み付き平均**を取ります。経路積分（自由エネルギー）の
結果によれば、最適制御分布はサンプリング分布の重み付けとして書けます:

$$
q^\*(\tau) \;\propto\; \exp\!\left(-\frac{S(\tau)}{\lambda}\right) p(\tau)
$$

したがって次の制御系列は重要度重み付き平均 $u^\* = \mathbb{E}_{q^\*}[u]$ を
サンプルから推定したものになります。$\lambda$ は温度で、$\lambda \to 0$ で
「最良のロールアウトを採用」（貪欲・高分散）、$\lambda \to \infty$ で全体平均
（更新量が消える）に対応します。

### 4.2 実装されているアルゴリズム

周期をまたいで保持されるノミナル系列 $\bar u \in \mathbb{R}^N$ を用いて、
毎周期:

1. **ウォームスタート（シフト）:** $i < N-1$ について $\bar u_i \leftarrow \bar u_{i+1}$。
2. **サンプリング:** $K$ 本の摂動 $\varepsilon^{(k)}_i \sim \mathcal N(0, \sigma^2)$ を
   生成し、$u^{(k)}_i = \mathrm{clamp}(\bar u_i + \varepsilon^{(k)}_i, \pm\delta_{\max})$。
3. **ロールアウト:** 各系列を速度一定のまま運動学モデルで刻み $T$ で展開:
   $$x \mathrel{+}= vT\cos\psi,\quad y \mathrel{+}= vT\sin\psi,\quad \psi \mathrel{+}= \frac{v\tan u_i}{L}T$$
4. **評価:**
   $$S_k = \sum_{i=0}^{N-1}\Big(w_e\,e_i^2 + w_\psi\,\theta_{e,i}^2 + w_\delta\,{u^{(k)}_i}^2\Big)$$
5. **重み付け**（数値安定化のため $S_{\min}$ を差し引いた softmax）:
   $$w_k = \frac{\exp\!\big(-(S_k - S_{\min})/\lambda\big)}{\sum_{j}\exp\!\big(-(S_j - S_{\min})/\lambda\big)}$$
6. **更新と適用:**
   $$\bar u_i \leftarrow \mathrm{clamp}\Big(\bar u_i + \sum_{k} w_k\,\varepsilon^{(k)}_i\Big),
   \qquad \delta = \mathrm{rate\_limit}(\bar u_0)$$

既定値: $K = 120$ サンプル、$N = 20$ ステップ、$T = 0.05\,\mathrm{s}$
（ホライズン 1.0 s）、$\sigma = 0.08\,\mathrm{rad}$、$\lambda = 1.0$、
$w_e = 8$、$w_\psi = 3$、$w_\delta = 0.3$、再現性のため乱数シードは 12345 固定。

### 4.3 標準的な定式化との差異

このコードを他所へ移植する前に把握しておくべき点:

- Williams らの情報理論的な重みには、重要度サンプリング補正に由来する制御項
  $\frac{\gamma}{\lambda}\bar u^\top \Sigma^{-1}\varepsilon$ が含まれます。本実装の
  重みは状態コストに対する素の softmax であり、操作量は代わりに
  $w_\delta u^2$ としてコストに入ります。いわゆる「vanilla MPPI」の簡略化で、
  更新にわずかなバイアスが入りますが、2 次コストでは十分安定に動きます。
- ロールアウト内では**速度を一定**に保ちます（縦方向ダイナミクスは
  サンプラ内でシミュレートしません）。
- 系列はロールアウト内でクランプされるため、操舵限界付近ではサンプリング分布が
  切断されます。

### 4.4 サンプル効率とノイズフロア

推定精度は実効サンプル数で決まります:

$$
\mathrm{ESS} = \frac{1}{\sum_k w_k^2} \in [1, K]
$$

コストのばらつきが $\lambda$ に比べて大きいと、ごく少数のロールアウトが重みを
独占し、更新は 1 周期あたり標準偏差 $\approx \sigma/\sqrt{\mathrm{ESS}}$ の
ランダムウォークのように振る舞います。実測の操舵レート RMS 174.8 °/s は
1 周期あたり $\approx 0.030\,\mathrm{rad}$ の変化に相当し、$K = 120$ に対して
$\mathrm{ESS} \sim \mathcal{O}(10)$ 程度であることを示唆します。このジッタが、
目に見えてノイジーな MPPI の軌跡（直線区間で $\pm 0.05\,\mathrm{m}$）と
4 手法中最悪の方位誤差 RMS 0.384° の正体です。$\lambda$ や $K$ を上げる、
あるいは $\sigma$ を下げると、ノイズと探索性・計算コストがトレードオフされます。

### 4.5 計算コストの内訳

1 制御周期あたり $K \times N = 2400$ ロールアウトステップを実行し、*各*ステップで
最大 $2W = 400$ 点の窓付き最近傍探索を行います:

$$
120 \times 20 \times 400 \approx 9.6\times10^5 \ \text{回の距離計算 / 周期}
$$

Pure Pursuit のタイミングから逆算される 1 回あたり約 0.6 ns を掛けると
≈ 570 µs となり、実測の 568 µs/周期とほぼ一致します。すなわち
**本実装の MPPI のコストは最近傍探索であり、ダイナミクスでも指数関数でも
ありません**。ロールアウトの探索窓を小さくする、あるいはインデックスを増分的に
追跡するだけで、並列化する前に 1 桁削減できます。なお $k$ について完全並列化
可能ですが、参照ビルドはシングルスレッドです。

### 4.6 MPPI が本領を発揮する場面

本ベンチマークは MPPI の長所を一切行使しません（コストは 2 次で微分可能、
モデルは滑らか、障害物も離散的な意思決定もない）。非凸・微分不可能・不連続な
コスト（衝突判定、走行可能領域マスク、接触ダイナミクスなど）は §3 の LQR
定式化では表現すらできません。それが、ここで幾何的手法の約 1600 倍の計算コストを
払って同等の精度しか得られない理由です。

---

## 5. ACC の縦方向制御則

[`acc_controller.hpp`](../include/path_tracking/acc_controller.hpp) — カットイン
デモで使用。横方向は Pure Pursuit のままです。

### 5.1 制御則

一定時間車間（constant time headway）方策に、相対速度と車間誤差の比例
フィードバックを加えたものです:

$$
g_{\mathrm{des}} = \max(g_{\min},\; h\,v_{\mathrm{ego}}),
\qquad
a = \mathrm{clamp}\Big(k_{\Delta v}\,(v_{\mathrm{lead}} - v_{\mathrm{ego}}) + k_g\,(g - g_{\mathrm{des}}),\; -a^{\max}_{\mathrm{dec}},\; a^{\max}_{\mathrm{acc}}\Big)
$$

$h = 2.0\,\mathrm{s}$、$g_{\min} = 5\,\mathrm{m}$、$k_{\Delta v} = 1.2$、
$k_g = 0.4$。先行車が `detection_range` = 80 m 以内にいない場合は経路速度追従
（§0.4）にフォールバックします。

### 5.2 閉ループ応答

車間誤差を $\varepsilon = g - h v_{\mathrm{ego}} - g_0$、相対速度を
$\Delta v = v_{\mathrm{lead}} - v_{\mathrm{ego}}$ とし、先行車速度一定・非飽和を
仮定すると:

$$
\begin{bmatrix}\dot\varepsilon \\ \Delta\dot v\end{bmatrix}
=
\begin{bmatrix} -h k_g & 1 - h k_{\Delta v} \\ -k_g & -k_{\Delta v}\end{bmatrix}
\begin{bmatrix}\varepsilon \\ \Delta v\end{bmatrix}
\;\Longrightarrow\;
s^2 + (k_{\Delta v} + h k_g)\,s + k_g = 0
$$

きれいな結果が得られます。固有振動数は車間ゲインのみ、減衰は
$k_{\Delta v} + h k_g$ の和のみで決まります:

$$
\omega_n = \sqrt{k_g} = 0.63\,\mathrm{rad/s},
\qquad
\zeta = \frac{k_{\Delta v} + h k_g}{2\sqrt{k_g}} = 1.58
$$

すなわち過減衰で、極は $-0.225$ と $-1.775\,\mathrm{s^{-1}}$。遅い極
（$\tau = 4.4\,\mathrm{s}$）が車間回復を支配するため、カットイン後およそ 13 秒で
先行車速度へ整定します。プロットに見える第 3 フェーズがこれです。$k_g$ を
上げると回復は $\sqrt{k_g}$ で速くなりますが $\zeta$ が下がり、$h$ を上げると
減衰は増えますが車間が広がります。

---

## 6. 総合比較

### 6.1 構造の比較

| | Pure Pursuit | Stanley | MPC | MPPI |
|---|---|---|---|---|
| 基準点 | 後輪車軸 | **前輪車軸** | 後輪車軸 | 後輪車軸 |
| 横方向誤差の直接利用 | なし（目標点経由で暗黙） | あり | あり | あり（コスト内） |
| 方位誤差の利用 | 暗黙 | あり | あり | あり |
| 曲率の扱い | $\ell_d/3$ のプレビュー（§1.5） | なし | フィードフォワード $\arctan(L\kappa)$ | ロールアウトに内包 |
| 将来経路の先読み | 幾何的にのみ | なし | なし（§3.1 参照） | あり、$NT = 1.0\,\mathrm{s}$ |
| 最適性 | なし | なし | LQ 最適解（厳密） | 確率的推定 |
| 円弧での定常偏差 | 0（§1.4） | $L^2\kappa/2$（§2.3） | 0（§3.5） | ほぼ 0（ノイジー） |
| 調整つまみ | 1（$k_v$） | 2（$k$, $k_{\mathrm{soft}}$） | 6（重み, $N$） | 6（$\sigma$, $\lambda$, $K$, $N$, 重み） |
| 1 周期あたりコスト | 0.35 µs | 0.37 µs | 1.52 µs | 568 µs |
| コストの主因 | 最近傍探索 | 最近傍探索 | Riccati, $O(Nn^3)$ | $KN$ 回の最近傍探索 |

### 6.2 実測値がなぜそうなるか

| 観測 | 説明 |
|---|---|
| Pure Pursuit がレート制限**なし**で最も滑らか（1.6 °/s） | 調整によらず $\zeta = 1/\sqrt2$（§1.3）。さらに入力が 7.8 m 先の点への方向なので、0.54 m の地図量子化を微分せずローパスする |
| Pure Pursuit のピーク誤差が最大（0.198 m） | $\ell_d/3$ の曲率プレビューにより進入クロソイドで早切りする（§1.5）。カーブではなく緩和区間の誤差 |
| Stanley のピーク（0.141 m）が $R = 25$ での定常偏差に一致 | 前輪車軸と後輪車軸の幾何的オフセット $L^2\kappa/2$（§2.3）。4 カーブすべてで 5〜7 % 以内に一致 |
| MPC が 5 倍精度（0.008 m RMS） | 厳密な曲率フィードフォワード＋閉ループ時定数 0.52 s、かつ指標と同じ点を制御している（§3.3, §3.5） |
| MPC の操舵レートが大きい（46.6 °/s） | LQR ではなく、最近傍点の量子化が $K_{0,2}v + K_{0,3}$ で増幅された周期 9 の鋸歯（§3.6） |
| MPPI の軌跡がノイジー（174.8 °/s、方位 RMS 0.384°） | $\mathrm{ESS} \sim 10$ に対するモンテカルロ更新ノイズ $\sigma/\sqrt{\mathrm{ESS}}$（§4.4） |
| MPPI が 568 µs/周期 | 1 周期あたり $9.6\times10^5$ 回の最近傍距離計算（§4.5） |

---

## 7. 参考文献

1. R. C. Coulter, *Implementation of the Pure Pursuit Path Tracking Algorithm*,
   CMU-RI-TR-92-01, 1992.
2. G. M. Hoffmann, C. J. Tomlin, M. Montemerlo, S. Thrun, *Autonomous Automobile
   Trajectory Tracking for Off-Road Driving: Controller Design, Experimental
   Validation and Racing*, ACC 2007.（Stanley 制御則と収束証明）
3. S. Thrun et al., *Stanley: The Robot that Won the DARPA Grand Challenge*,
   Journal of Field Robotics, 2006.
4. J. M. Snider, *Automatic Steering Methods for Autonomous Automobile Path
   Tracking*, CMU-RI-TR-09-08, 2009.（幾何的手法とモデルベース手法の比較、
   §3.1 の横方向誤差状態モデル）
5. G. Williams, A. Aldrich, E. Theodorou, *Model Predictive Path Integral
   Control: From Theory to Parallel Computation*, JGCD, 2017; G. Williams
   et al., *Aggressive Driving with Model Predictive Path Integral Control*,
   ICRA 2016.
6. B. D. O. Anderson, J. B. Moore, *Optimal Control: Linear Quadratic Methods*,
   1990.（Riccati 漸化式と DARE 解への収束）
