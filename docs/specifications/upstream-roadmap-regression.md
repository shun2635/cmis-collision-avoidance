# upstream Roadmap 回帰基盤

## 1. 目的

この文書は、`external/RVO2/examples/Roadmap.cc` を参照して追加した回帰基盤を記録する。  
目的は、visibility graph と waypoint guidance を含む upstream example を Python 実装で再現し、obstacle-heavy な条件で ORCA の劣化を検出できるようにすることである。

## 2. 出典

- 参照元: `external/RVO2/examples/Roadmap.cc`
- upstream: <https://github.com/snape/RVO2>
- upstream license: Apache License 2.0

## 3. 現在の構成

| 種別 | パス | 内容 |
| --- | --- | --- |
| scenario / roadmap builder | `src/cmis_ca/regression/upstream_roadmap.py` | Roadmap 条件を `Scenario`、visibility graph、goal index として組み立てる |
| 実行スクリプト | `scripts/compare_upstream_roadmap.py` | metric の出力 |
| 回帰テスト | `tests/regression/test_upstream_roadmap.py` | setup 確認と定性的回帰チェック |

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
- goal vertex は 4 隅 `(-75, -75)`, `(75, -75)`, `(-75, 75)`, `(75, 75)`
- obstacle 周辺の roadmap vertex 16 個を upstream と同じ座標で配置
- preferred velocity は各 step で「可視な vertex のうち、goal までの推定距離が最短なもの」へ向けて再計算する

## 5. Python 側での扱い

- `Roadmap.cc` は direct goal ではなく roadmap guidance を使うため、現行 `goal_position` の直線追従 API だけでは再現できない
- そのため `upstream_roadmap.py` では、regression helper の中で visibility graph と Dijkstra 距離表を構築する
- preferred velocity 更新は `Simulator.set_preferred_velocity()` を使い、各 step の直前に helper 側で実施する
- upstream の時刻依存乱数は回帰の再現性が悪いため、現行実装では固定 seed による deterministic perturbation を使う

## 6. visibility の現在仕様

- public な roadmap planner や `queryVisibility()` API は追加していない
- regression helper の中で、query segment と obstacle edge の交差判定、および clearance 半径つき最短距離判定を行う
- この visibility helper は Roadmap regression 専用であり、core の正式 API ではない

## 7. 現在の比較観点

完全な位置一致ではなく、以下の定性的 metric を回帰条件として使う。

- average goal distance が初期値から減少していること
- agent 間最短距離が半径 2 倍を大きく下回らないこと
- 重心のずれが暴走していないこと
- 速度が最大速度を超えないこと
- goal 到達前でも roadmap guidance により進行が継続していること

## 8. 現在の回帰条件

`tests/regression/test_upstream_roadmap.py` では、32 step 実行後に以下を確認する。

- `goal_distance_reduction > 7.0`
- `minimum_pair_distance > 8.0`
- `centroid_distance < 1.0`
- `max_speed <= 2.0`
- `reached_goal == False`

これらは 2026-03-12 時点の現行実装を基準にした定性的な閾値である。

## 9. 実測メモ

`scripts/compare_upstream_roadmap.py` を 64 step で実行したときの現行値は以下。

- `reached_goal = False`
- `reached_goal_count = 0`
- `average_goal_distance ≈ 197.209068`
- `goal_distance_reduction ≈ 15.393879`
- `minimum_pair_distance ≈ 7.308300`
- `centroid_distance ≈ 0.770311`
- `max_speed ≈ 1.000097`
- `passage_agent_count = 0`

## 10. 制約

- `Roadmap.cc` の visibility graph は regression helper 内の専用実装であり、core へ一般化していない
- perturbation は upstream の time-seeded random ではなく deterministic seed を使う
- 近傍探索は `NaiveNeighborSearch` のため、長時間実行の性能と順序は upstream kd-tree と一致しない
- 現時点の test は 32 step の早期挙動固定であり、goal 到達完了までは回していない
