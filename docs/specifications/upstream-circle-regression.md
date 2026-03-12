# upstream Circle 回帰基盤

## 1. 目的

この文書は、`external/RVO2/examples/Circle.cc` を参照して作成した比較用回帰基盤を記録する。  
目的は upstream 完全一致を保証することではなく、将来の ORCA 実装変更で挙動が崩れていないかを継続的に確認できる入口を固定することである。

obstacle を含む companion scenario については [upstream-blocks-regression.md](upstream-blocks-regression.md) を参照する。

## 2. 出典

- 参照元: `external/RVO2/examples/Circle.cc`
- upstream: <https://github.com/snape/RVO2>
- upstream license: Apache License 2.0

## 3. 現在の構成

| 種別 | パス | 内容 |
| --- | --- | --- |
| 比較用シナリオ | `scenarios/upstream_circle.yaml` | 250 体を半径 200 の円周上へ等間隔配置し、profile に ORCA defaults と goal-stop 条件を保持 |
| 回帰 helper | `src/cmis_ca/regression/upstream_circle.py` | scenario-defined goals と upstream-style stop 条件を使って metric を集計 |
| 実行スクリプト | `scripts/compare_upstream_circle.py` | metric の出力 |
| 回帰テスト | `tests/regression/test_upstream_circle.py` | 条件確認と定性的回帰チェック |

## 4. upstream から取り込んだ条件

現在固定している条件は以下。

- `time_step = 0.25`
- agent 数 `250`
- 初期配置半径 `200`
- `neighbor_dist = 15`
- `max_neighbors = 10`
- `time_horizon = 10`
- `time_horizon_obst = 10`
- agent 半径 `1.5`
- agent 最大速度 `2.0`
- 上記 ORCA defaults は各 agent の `profile` に記述する
- goal は各 agent の初期位置の antipodal point
- `scenarios/upstream_circle.yaml` に `goal_position` と `preferred_speed=1.0` を明示する
- preferred velocity は `Simulator` が各 step で goal 方向へ再計算する
- upstream `do ... while (!reachedGoal())` に合わせ、既定実行は全 agent が各自の `radius` 以内へ到達するまで継続する
- `scenarios/upstream_circle.yaml` では `steps: 0` と `stop_when_all_agents_reach_goals: true` を使ってこの停止条件を表す

## 5. 現在の比較観点

完全な位置一致ではなく、以下の定性的 metric を回帰条件として使う。

- 平均半径が減少していること
- agent 間最短距離が半径 2 倍を下回らないこと
- 重心が原点近傍に残ること
- 速度が最大速度を超えないこと
- antipodal な agent 対が対称性を保つこと

## 6. 現在の回帰条件

`tests/regression/test_upstream_circle.py` では、8 step 実行後に以下を確認する。

- `average_radius < 198.5`
- `minimum_pair_distance > 3.0`
- `centroid_distance < 1.0e-9`
- `max_speed <= 2.0`
- `max_antipodal_error < 1.0e-8`

これらは 2026-03-12 時点の現行実装を基準にした定性的な閾値である。

## 7. 実測メモ

`run_upstream_circle_regression()` は既定では upstream と同じく goal 到達まで回す。  
一方、pytest の回帰は実行時間を抑えるため `steps=8` の fixed-step で固定している。

2026-03-12 時点の `steps=8` 実測は以下。

- `average_radius < 198.5`
- `minimum_pair_distance > 3.0`
- `centroid_distance < 1.0e-9`
- `max_speed <= 2.0`
- `max_antipodal_error < 1.0e-8`

## 8. 制約

- goal API は scenario schema と `Simulator` に導入済みである
- 比較は upstream 完全一致ではなく、早期ステップの定性的挙動固定にとどまる
- full run は `NaiveNeighborSearch` のため計算量が大きく、日常の自動テストでは固定 step 回帰を使う
- 近傍探索は `NaiveNeighborSearch` のため、大規模 step 数では計算量が大きい
