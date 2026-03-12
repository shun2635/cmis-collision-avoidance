# ORCA neighbor search parity

## 1. 目的

この文書は、upstream `RVO2` の `Agent::computeNeighbors` と `KdTree` に対して、現行 Python 実装の `NaiveNeighborSearch` がどこまで追従しているかを固定する。

## 2. 参照元

- `external/RVO2/src/Agent.cc`
- `external/RVO2/src/KdTree.cc`
- `src/cmis_ca/core/neighbor_search.py`
- `src/cmis_ca/algorithms/orca/algorithm.py`

## 3. 追従済み項目

### 3.1 agent neighbor

- agent 探索範囲は `neighbor_dist^2`
- 距離境界は upstream と同じ strict `< rangeSq`
- `max_neighbors` 到達後は末尾距離で range を更新する
- 挿入順は `insertAgentNeighbor()` と同じ stable insertion を使う

### 3.2 obstacle neighbor

- obstacle 探索範囲は `time_horizon_obst * max_speed + radius`
- obstacle 候補は directed edge の右側にある agent だけを採用する
- 線分距離の境界は strict `< rangeSq`
- 挿入順は `insertObstacleNeighbor()` と同じ stable insertion を使う

### 3.3 ORCA 統合

- `ORCAAlgorithm.step()` は agent range と obstacle range を分けて `NeighborSearch` へ渡す
- `NeighborSet` は距離付き結果を返し、制約生成側は採用済み近傍だけを読む

## 4. 回帰で固定した代表ケース

- `distance == neighbor_dist` の agent を除外するケース
- 同距離 agent の tie で stable order を保つケース
- directed obstacle edge の左側を除外するケース
- agent range より長い obstacle range を使うケース

## 5. 現時点で残す差分

- 実装は依然として `O(n^2)` の `NaiveNeighborSearch` であり、upstream の kd-tree ではない
- kd-tree の探索順そのものは再現していないため、幾何的な完全同率ケースで最終順序が一致する保証はない
- obstacle tree の visibility 剪定は持たないため、性能特性は upstream と異なる

## 6. `NaiveNeighborSearch` を残す理由

- 研究室内で読みやすく、数式と制御分岐を追いやすい
- ORCA の挙動再現に必要な range と insertion semantics は反映できる
- kd-tree は issue として後段へ分離しやすく、まず correctness を docs と tests で固定できる

## 7. 結論

neighbor search は、ORCA の回帰に効く主要 semantics を upstream ベースで揃えた。  
残る差分は主にデータ構造と探索順由来の性能・同率ケースであり、現在の main gap は regression suite のさらなる強化へ移っている。
