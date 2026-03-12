# Issue 0019: ORCA の neighbor search parity を監査する

- ステータス: completed
- 優先度: high
- 関連文書:
  - [docs/specifications/orca-parity-gap-review.md](../specifications/orca-parity-gap-review.md)
  - [docs/specifications/orca-agent-solver-parity.md](../specifications/orca-agent-solver-parity.md)
  - [external/RVO2/src/KdTree.cc](../../external/RVO2/src/KdTree.cc)
  - [external/RVO2/src/Agent.cc](../../external/RVO2/src/Agent.cc)

## 背景

agent-agent / obstacle line と solver は upstream ベースで移植済みだが、入力となる近傍集合は `NaiveNeighborSearch` に依存している。  
完全再現を目指すなら、neighbor 採用順と打ち切り条件の差分を監査する必要がある。

## 目的

neighbor semantics を upstream と比較し、どこまで合わせるか、どこを受容差分とするかを明文化する。

## スコープ

- agent neighbor search の差分整理
- obstacle neighbor search の差分整理
- 必要なら reference 実装や回帰テストを追加する
- docs を更新する

## 非スコープ

- kd-tree の最適化そのもの
- proxemic / cnav 対応

## 完了条件

- neighbor search の差分が docs に整理されている
- ORCA の回帰に効く近傍ケースがテスト化されている
- `NaiveNeighborSearch` を残す理由または置き換える理由が説明できる

## 実施メモ

- `NeighborSearch.find_neighbors()` に `obstacle_range` を追加し、agent range と obstacle range を分離した
- `NaiveNeighborSearch` の agent 採用を strict 境界と stable insertion に変更した
- obstacle 採用を directed edge の右側だけへ絞り、strict 境界と stable insertion に変更した
- `ORCAAlgorithm.step()` で obstacle range を `time_horizon_obst * max_speed + radius` として渡すようにした
- 回帰テストを追加し、結果を [../specifications/orca-neighbor-search-parity.md](../specifications/orca-neighbor-search-parity.md) に記録した
