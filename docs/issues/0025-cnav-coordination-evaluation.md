# Issue 0025: CNav の coordination evaluation を実装する

- ステータス: completed
- 優先度: high
- 関連文書:
  - [docs/algorithms/cnav.md](../algorithms/cnav.md)
  - [docs/issues/0022-cnav-initial-implementation.md](0022-cnav-initial-implementation.md)

## 背景

CNav の本体は、neighbor の constraint 状態を見て action を short-horizon simulation で評価し、最良の `v_intent` を選ぶ部分にある。  
この層が入って初めて、ORCA の上に CNav の coordination layer が成立する。

## 目的

`GetMostConstrainedNeighs`、`SimMotion`、`R_ga`、`R_ca`、`R_a` を実装し、論文どおりの action evaluation を追加する。

## スコープ

- constrained neighbor ranking
- `自分より goal に近い neighbor` フィルタ
- `T=2` の short-horizon simulation
- goal-progress 成分
- constraint-reduction 成分
- reward 最大化による action 選択

## 非スコープ

- 広い scenario 再現
- GUI 比較
- action set の一般化

## 完了条件

- constrained neighbor を順位付けできる
- 各 action を simulation で評価できる
- `R_a = (1 - γ) * R_ga + γ * R_ca` で action を選べる
- `CNavAlgorithm.step()` が action 選択を反映して ORCA helper を呼べる

## 想定成果物

- `src/cmis_ca/algorithms/cnav/coordination.py`
- `src/cmis_ca/algorithms/cnav/algorithm.py` の本体更新

## 作業メモ

- runtime 専用 state は `algorithms/cnav/` に閉じ込める
- 初回実装では top-k と gamma を parameter 化しつつ、default は論文寄りに寄せる

## 実施メモ

- `src/cmis_ca/algorithms/cnav/coordination.py` を追加し、`rank_constrained_neighbors(...)`、`evaluate_action(...)`、`select_best_action(...)` を実装した
- constrained neighbor ranking は `自分の goal に対して自分より前にいる neighbor` のみを対象にし、`||v_intent - v_observed||` の大きい順に並べる
- action evaluation は `T=2` の short-horizon simulation で `R_ga` と `R_ca` を計算し、`R_a` 最大の action を選ぶ
- `WorldSnapshot` / `SnapshotAgent` へ `goal_position` を通し、CNav が goal 依存の判定をできるようにした
- `tests/algorithms/test_cnav.py` に ranking と action evaluation の test を追加した
