# path_tracking_oss

[English](README.md) | **日本語**

**車両の経路追従（path tracking）** を題材にしたオープンソースの C++17 サンプル
です。**4 つの古典的な経路追従アルゴリズム**を、同一の基準経路・同一の運動学的
車両モデル上で実装・**比較**します。

| アルゴリズム | 系統 | 考え方 |
|-------------|------|--------|
| **Pure Pursuit** | 幾何的 | 経路上の先読み点（look-ahead point）を追いかける。 |
| **Stanley** | 幾何的（前輪軸） | 前輪軸で方位誤差＋横方向誤差（cross-track error）を補正する。 |
| **MPC** | 最適化 | 横方向誤差ダイナミクスに対する線形時変 MPC。後退 Riccati 漸化式で解く。 |
| **MPPI** | サンプリング | Model Predictive Path Integral: 多数のノイズ付き操舵系列を展開し、コスト重み付き平均を取る。 |

Pure Pursuit → Stanley → MPC → MPPI という流れは、*幾何的ヒューリスティクス*
から*モデル予測最適化*、そして*サンプリングに基づく最適化*へという歴史的な
進展を反映しています。

### 4 つの操舵則（概観）

$$
\delta_{\mathrm{PP}} = \arctan\!\left(\frac{2L\sin\alpha}{\ell_d}\right)
\qquad
\delta_{\mathrm{Stanley}} = k_\psi\theta_e - \arctan\!\left(\frac{k\,e_f}{k_{\mathrm{soft}} + v}\right)
$$

$$
\delta_{\mathrm{MPC}} = \arctan(L\kappa) - K_0 z, \quad K_k = \frac{B^\top P_{k+1}A}{R + B^\top P_{k+1}B}
\qquad
\delta_{\mathrm{MPPI}} = \bar u_0 + \sum_k w_k\varepsilon^{(k)}_0,\quad w_k \propto e^{-S_k/\lambda}
$$

導出、閉ループ解析、パラメータ表、理論値と実測トレースの突き合わせは
**[`docs_ja/algorithms.md`](docs_ja/algorithms.md)** にまとめてあります。

## リポジトリ構成

```
path_tracking_oss/
├── CMakeLists.txt
├── README.md
├── include/path_tracking/      # ヘッダオンリーライブラリ
│   ├── types.hpp               #   幾何型、Path、ControlCommand
│   ├── vehicle.hpp             #   運動学的自転車モデル
│   ├── path_io.hpp             #   CSV 読み込み、速度プロファイル、最近傍探索
│   ├── controller.hpp          #   抽象 Controller インタフェース
│   ├── pure_pursuit.hpp        #   Pure Pursuit コントローラ
│   ├── stanley.hpp             #   Stanley コントローラ
│   ├── mpc.hpp                 #   モデル予測制御（Eigen）
│   ├── mppi.hpp                #   Model Predictive Path Integral
│   ├── acc_controller.hpp      #   ACC / 車間追従（カットインデモ）
│   └── simulator.hpp           #   閉ループシミュレーション＋指標算出
├── src/
│   ├── main.cpp                # 比較ベンチマーク
│   └── cutin_demo.cpp          # ACC カットイン追従デモ
├── tests/path_tracking_tests.cpp   # 回帰テスト（ctest）
├── tools/                      # 経路生成・matplotlib による図／アニメーション生成スクリプト
├── data/reference_route.csv     # 基準経路（約 1.0 km、合成）
├── docs_en/                    # アルゴリズム詳解・レポート・図（英語）
│   ├── algorithms.md           #   導出、閉ループ解析、パラメータ
│   └── comparison_report.md    #   ベンチマーク結果と図
└── docs_ja/                    # 同上（日本語）
```

## 依存関係

- C++17 コンパイラ（GCC、Clang、MSVC）。
- CMake ≥ 3.16。
- **Eigen3**（≥ 3.3）— MPC コントローラの行列演算に使用。
  - Ubuntu/Debian: `sudo apt install libeigen3-dev`
  - macOS: `brew install eigen`
  - vcpkg: `vcpkg install eigen3` の後、
    `-DCMAKE_TOOLCHAIN_FILE=<vcpkg>/scripts/buildsystems/vcpkg.cmake` を指定。
- Python 3 と `matplotlib`（任意の図生成スクリプト用のみ）。

Pure Pursuit、Stanley、MPPI は C++ 標準ライブラリのみに依存します。Eigen を
リンクするのは MPC だけです。

## クイックスタート（Windows / PowerShell）

単一のヘルパースクリプト — **`run_simulation.ps1`** — が CMake 構成、ビルド、
シミュレーション実行、アニメーション起動を 1 ステップで行います。これは旧来の
`build_and_run.ps1` / `build_cutin_demo.ps1` スクリプトを置き換えたものです。
実行するデモは `-Target` で選びます:

```powershell
# 両方のデモをビルドして実行（既定）
.\run_simulation.ps1

# 経路追従の比較ベンチマークのみ
.\run_simulation.ps1 -Target compare

# ACC カットイン追従デモのみ（4 パネルプロット＋アニメーション）
.\run_simulation.ps1 -Target cutin
```

オプション:

| フラグ | 意味 |
|--------|------|
| `-Target compare\|cutin\|all` | ビルド／実行するデモ（既定 `all`） |
| `-BuildType Release\|Debug` | ビルド構成（既定 `Release`） |
| `-Reconfigure` | CMake を強制的に再構成する |
| `-NoAnimate` | ビルド＋実行のみ。matplotlib のアニメーション／プロットをスキップ |
| `-Save FILE` | アニメーションをウィンドウ表示せず `FILE`（MP4/GIF）に保存 |

```powershell
# Debug でビルドし、アニメーションをスキップ
.\run_simulation.ps1 -BuildType Debug -NoAnimate

# ウィンドウ表示の代わりにアニメーションをファイルに保存
.\run_simulation.ps1 -Target compare -Save demo.mp4
```

ACC カットインデモの詳細は
[ACC カットイン追従デモ](#acc-カットイン追従デモ)を参照してください。

手動・クロスプラットフォームのビルドは以下を参照してください。

## ビルド

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
```

## 実行

```bash
cd build
./path_tracking_compare [route.csv] [output_dir]
```

- `route.csv` — 基準経路（既定 `data/reference_route.csv`）。
- `output_dir` — トレース CSV の出力先（既定: カレントディレクトリ）。

プログラムは 4 つのコントローラすべてを実行し、アルゴリズムごとに 1 つのトレース
CSV（`trace_<Algorithm>.csv`）と `comparison_summary.csv` を出力し、性能比較表を
並べて表示します。

### 基準経路 CSV フォーマット

ヘッダ行に続いて、1 行 1 点:

```
x_m, y_m, lat, lon, yaw, curvature
```

使用されるのは `x_m`、`y_m`、`yaw`、`curvature` のみです。弧長と曲率制限付きの
基準速度プロファイルは自動的に導出されます。

## 図

```bash
python tools/plot_results.py build data/reference_route.csv
```

結果ディレクトリに以下を生成します:

- `fig_trajectory.png` — 基準経路に重ねた XY 軌跡。
- `fig_error_timeseries.png` — 横方向誤差と操舵の時間変化。
- `fig_metrics.png` — 集約指標の棒グラフ。

4 つのコントローラすべてを基準経路上で比較したアニメーション
（`tools/animate_results.py`）:

![経路追従の比較アニメーション](docs_ja/animation_demo.gif)

## 結果

同一の約 1.0 km の経路・運動学的自転車モデルで 4 つのコントローラを実行し、同一の
縦方向速度則を共有することで、比較が*操舵*を切り分けられるようにしています
（詳細レポート: [`docs_ja/comparison_report.md`](docs_ja/comparison_report.md)）:

| アルゴリズム | CTE RMS [m] | CTE 最大 [m] | 方位 RMS [deg] | 操舵レート RMS [deg/s] | 計算時間 [µs/周期] |
|-------------|-------------|--------------|----------------|------------------------|--------------------|
| Pure Pursuit | 0.038 | 0.198 | 0.235 | **1.6** | **0.35** |
| Stanley | 0.040 | 0.141 | 0.151 | 14.3 | 0.37 |
| MPC | **0.008** | **0.030** | **0.115** | 46.6 | 1.52 |
| MPPI | 0.036 | 0.138 | 0.384 | 174.8 | 568 |

この運動学的な試験では **MPC が最も小さい横方向誤差**（他手法の約 1/5）を達成し、
**Pure Pursuit が最も滑らかな操舵**を最小の計算コストで実現します。MPPI は
そのサンプリング探索のために、幾何的手法の 1 周期あたり約 1600 倍のコストを
支払います。

## 指標

各実行についてシミュレータは以下を報告します:

| 指標 | 意味 |
|------|------|
| `cte_rms` / `cte_max` | 横方向誤差の RMS ／ ピーク [m] |
| `heading_rms` | 方位誤差の RMS [deg] |
| `steer_rms` | 操舵角の RMS [deg] |
| `steer_rate_rms` | 操舵レートの RMS [deg/s] — アクチュエーションの滑らかさ |
| `avg_speed` | 平均速度 [km/h] |
| `compute_us_per_cycle` | 1 周期あたりのコントローラ実時間 [µs] |

## ACC カットイン追従デモ

純粋な経路追従に加えて、本リポジトリには **アダプティブクルーズコントロール
（ACC）のカットインシナリオ**（`src/cutin_demo.cpp` ＋
`include/path_tracking/acc_controller.hpp`）が含まれます。自車は基準経路を巡航し、
途中で低速の先行車が前方に割り込みます。ACC コントローラは Pure Pursuit の横方向
制御を、車間追従の縦方向制御則でラップします:

```
gap_desired = max(min_gap, time_gap * v_ego)
a = kp_dv * (v_lead - v_ego) + kp_gap * (gap - gap_desired)
a = clamp(a, -decel_max, accel_max)
```

これにより 3 つの目に見えるフェーズが生じます: 自由巡航 → カットインでの急制動
→ 先行車速度での定常的な車間追従。

```bash
# ビルド（cutin_demo ターゲットは通常の CMake ビルドに含まれる）
cmake --build build --target cutin_demo

# 実行 — cutin_trace.csv を出力
./build/cutin_demo [route.csv] [out_dir]

# 可視化／アニメーション化
python tools/plot_cutin_demo.py    cutin_trace.csv
python tools/animate_cutin_demo.py cutin_trace.csv --save cutin.gif
```

## テスト

外部フレームワーク不要のヘッダオンリー回帰テストが CTest に登録されています:

```bash
cmake --build build --target path_tracking_tests
ctest --test-dir build --output-on-failure
```

`cross_track_error` の符号・大きさ、窓付きの `nearest_index` 探索が全走査と一致
すること、`assign_speed_profile` が曲率の増加に伴って基準速度を下げること、
Pure Pursuit が 1 m の横方向オフセットから直線へ収束することを検証します。

## 基準経路データ

[`data/reference_route.csv`](data/reference_route.csv) は合成経路（約 1.0 km）です。
実道路の線形と同じく、直線と定曲率円弧をクロソイド緩和曲線で接続して構成しており、
R = 200 / 30 / 150 / 25 m の 4 つのカーブを含みます。生成は
[`tools/generate_reference_route.py`](tools/generate_reference_route.py) が行うため、
同スクリプト冒頭の曲率プロファイルを編集して再実行すれば任意の経路に差し替えられます。
第三者の地図データは一切使用していません。列フォーマットの詳細は
[`data/README.md`](data/README.md) を参照してください。

## ライセンス

Apache-2.0 ライセンス — 各ソースファイルの `SPDX-License-Identifier` ヘッダを参照して
ください。
