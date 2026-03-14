# CNav 初期実装仕様

## 1. 目的

この文書は、issue `0022` `0023` `0024` `0025` `0026` を通じて追加した CNav 初期実装の実装事実を固定する。  
対象は `2026-03-13` 時点の Python 実装であり、論文の完全再現ではなく、`ORCA 上に載る coordination layer の最小導入` を記録する。

## 2. 実装位置

| 種別 | パス | 役割 |
| --- | --- | --- |
| algorithm package | `src/cmis_ca/algorithms/cnav/` | CNav 固有実装 |
| parameter | `src/cmis_ca/algorithms/cnav/parameters.py` | `CNavParameters` |
| action set | `src/cmis_ca/algorithms/cnav/actions.py` | 論文既定の 8 action 生成 |
| coordination | `src/cmis_ca/algorithms/cnav/coordination.py` | neighbor ranking、short-horizon evaluation、reward 計算 |
| algorithm | `src/cmis_ca/algorithms/cnav/algorithm.py` | intent cache 管理、action 更新、ORCA 呼び出し |
| ORCA helper | `src/cmis_ca/algorithms/orca/agent_solver.py` | arbitrary intended velocity に対する per-agent ORCA solve |
| scenario | `scenarios/cnav_queue.yaml` | CNav 用最小外部 scenario |

## 3. 現在の位置付け

- `cnav` は `AlgorithmRegistry` に登録済みで、CLI から選択できる
- CNav は低レベル衝突回避を自前で持たず、`compute_orca_velocity(...)` を使って ORCA を再利用する
- 現在の実装は `action-selection layer の最小再現` であり、論文の広い実験セットや比較ベンチマークは未実装

## 4. 現在の入出力

### 4.1 追加された algorithm 名

- `cnav`

### 4.2 CNav が要求する入力条件

- 全 agent に `goal_position` が必要
- `WorldSnapshot.agents[*]` は `goal_position` を含む
- `time_step > 0`

goal が欠けると、CNav の ranking と action evaluation は `ValueError` とする。

### 4.3 パラメータ

`CNavParameters` の現行フィールド:

- `coordination_factor`
- `simulation_horizon_steps`
- `action_update_interval`
- `update_every_step`
- `top_k_constrained_neighbors`
- `action_speed`
- `beta_degrees`

既定値は次のとおり。

- `coordination_factor = 0.8`
- `simulation_horizon_steps = 2`
- `action_update_interval = 0.2`
- `update_every_step = false`
- `top_k_constrained_neighbors = 3`
- `action_speed = 1.5`
- `beta_degrees = 45.0`

比較用の named preset として、`create_cnav_parameters("legacy-forpaper-comparison")` も追加されている。  
これは mainline default を変えずに、`coordination_factor = 0.9`、`simulation_horizon_steps = 3`、`update_every_step = true` をまとめて選ぶための比較用 preset である。

## 5. action set

現在の action set は論文 Figure 2 の 8 action 固定である。

- `0°`
- `+β`
- `-β`
- `+90°`
- `-90°`
- `180°`
- `180° + β`
- `180° - β`

`β` は `beta_degrees` で与える。  
各 action は goal 方向ベクトルを基準にし、speed は `action_speed` を使う。

## 6. intent cache と action 更新

- CNav は agent ごとに `intended_velocity` を cache する
- cache entry には `last_update_time` を持つ
- `update_every_step = false` のとき、action 更新判定は `global_time - last_update_time >= action_update_interval` で行う
- `update_every_step = true` のとき、毎 step action を再評価する
- 同一 step 内では、step 開始時点の communicated intent を固定参照する

このため、agent の処理順は communication 内容に影響しない。

## 7. constrained neighbor ranking

`rank_constrained_neighbors(...)` の現行仕様:

- 対象は `neighbor_search` が返した近傍 agent に限る
- `自分の goal に対して、自分より goal に近い neighbor` だけを採用する
- constraint の強さは `||v_intent - v_observed||` で評価する
- 降順ソートし、同点時は `agent index` 昇順とする

## 8. action evaluation

`evaluate_action(...)` の現行仕様:

- horizon は `simulation_horizon_steps`
- 現行既定は `T = 2`
- simulation 対象は `self + ranked neighbors`
- self の candidate action は評価中の action を使う
- neighbor 側は communicated intent を ORCA の optimization velocity として使う

reward 構成:

- goal-progress 成分 `R_ga`
- constrained-reduction 成分 `R_ca`
- 総合 reward `R_a = (1 - gamma) * R_ga + gamma * R_ca`

`select_best_action(...)` は action set 全体を評価し、最大 `R_a` の action を返す。

## 9. ORCA との接続

- `CNavAlgorithm.step()` は各 agent の intended velocity を決めた後、`compute_orca_velocity(...)` を呼ぶ
- ORCA 側の neighbor search、constraint build、solver はそのまま再利用する
- CNav は ORCA 実装を重複して持たない

## 10. CLI と scenario

現在の実行例:

```bash
poetry run cmis-ca run --algorithm cnav --scenario scenarios/cnav_queue.yaml
poetry run cmis-ca run --algorithm cnav --steps 4
poetry run cmis-ca visualize --algorithm cnav --scenario scenarios/cnav_queue.yaml
```

`run_demo()` の built-in `circle-demo` も goal を持つため、`cnav` で実行可能である。

## 11. テスト

現在の主な自動確認:

- `tests/algorithms/test_cnav.py`
  - action set 順序
  - parameter default
  - intent cache の保持と更新
  - constrained neighbor ranking
  - action evaluation
  - polite action 選択
- `tests/cli/test_main.py`
  - CLI から `--algorithm cnav` を受け取れること
- `tests/test_simulator_smoke.py`
  - `create_algorithm("cnav")` で最小実行できること

## 12. 制約

- 現在の実装は論文の完全再現ではない
- action set は固定で、一般化は未実装
- communication 遅延や損失はモデル化していない
- 大規模 benchmark / regression は未実装
- CNav 専用可視化や比較 metric も未実装
