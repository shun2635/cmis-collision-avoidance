# upstream Blocks 回帰基盤

## 1. 目的

この文書は、`external/RVO2/examples/Blocks.cc` を参照して追加した回帰基盤を記録する。  
目的は、obstacle を含む upstream 由来 scenario で ORCA 実装の劣化を検出できるようにすることである。

## 2. 出典

- 参照元: `external/RVO2/examples/Blocks.cc`
- upstream: <https://github.com/snape/RVO2>
- upstream license: Apache License 2.0

## 3. 現在の構成

| 種別 | パス | 内容 |
| --- | --- | --- |
| scenario builder | `src/cmis_ca/regression/upstream_blocks.py` | Blocks 条件を Python の `Scenario` として組み立てる |
| 実行スクリプト | `scripts/compare_upstream_blocks.py` | metric の出力 |
| 回帰テスト | `tests/regression/test_upstream_blocks.py` | setup 確認と定性的回帰チェック |

## 4. upstream から取り込んだ条件

現在固定している条件は以下。

- `time_step = 0.25`
- agent 数 `100`
- 4 隅に 25 体ずつ配置
- `neighbor_dist = 15`
- `max_neighbors = 10`
- `time_horizon = 5`
- `time_horizon_obst = 5`
- agent 半径 `2.0`
- agent 最大速度 `2.0`
- goal は対角側の corner goal
- 4 個の polygon obstacle を upstream と同じ頂点順で配置
- preferred velocity は helper 側で step ごとに再計算し、微小 perturbation も加える

## 5. scenario を code-generated にしている理由

Blocks は 100 体の反復配置と 4 グループの goal を持つため、YAML へ展開すると可読性が落ちる。  
また、upstream は scenario 外で preferred velocity と perturbation を更新するため、現時点では scenario builder と helper を Python に置き、upstream 条件と対応づけやすい形を優先している。

## 6. 現在の比較観点

完全な位置一致ではなく、以下の定性的 metric を回帰条件として使う。

- average goal distance が初期値から十分減少していること
- agent 間最短距離が半径 2 倍を大きく下回らないこと
- 重心が原点近傍に残ること
- 速度が最大速度を大きく超えないこと
- central region に進入した agent が現れること

## 7. 現在の回帰条件

`tests/regression/test_upstream_blocks.py` では、64 step 実行後に以下を確認する。

- `goal_distance_reduction > 10.0`
- `minimum_pair_distance > 6.0`
- `centroid_distance < 1.0e-4`
- `max_speed <= 2.0`
- `central_agent_count >= 4`

これらは 2026-03-12 時点の現行実装を基準にした定性的な閾値である。

## 8. 実測メモ

`scripts/compare_upstream_blocks.py` を 64 step で実行したときの現行値は以下。

- `average_goal_distance ≈ 196.647580`
- `goal_distance_reduction ≈ 15.955368`
- `minimum_pair_distance ≈ 9.445364`
- `centroid_distance ≈ 0.000004`
- `max_speed ≈ 1.000098`
- `central_agent_count = 4`

## 9. 制約

- perturbation は upstream の time-seeded random ではなく deterministic seed を使う
- roadmap は使わず、`Blocks.cc` 同様に direct goal だけを使う
- 近傍探索は `NaiveNeighborSearch` のため、長時間実行の性能と順序は upstream kd-tree と一致しない
