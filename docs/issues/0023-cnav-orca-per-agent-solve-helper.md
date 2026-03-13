# Issue 0023: CNav 向けに ORCA の per-agent solve helper を抽出する

- ステータス: completed
- 優先度: high
- 関連文書:
  - [docs/algorithms/cnav.md](../algorithms/cnav.md)
  - [docs/issues/0022-cnav-initial-implementation.md](0022-cnav-initial-implementation.md)
  - [docs/specifications/orca-agent-solver-parity.md](../specifications/orca-agent-solver-parity.md)

## 背景

CNav は、agent ごとに内部保持している `v_intent` を ORCA に渡して最終速度 `v_new` を得る必要がある。  
現状の ORCA 実装は `ORCAAlgorithm.step()` の中で constraint build と solver 呼び出しを閉じており、CNav から安全に再利用しにくい。

## 目的

ORCA の per-agent solve を helper として切り出し、`optimization_velocity` を明示的に渡せる形へ整理する。  
これにより、CNav が ORCA 実装を重複せずに再利用できるようにする。

## スコープ

- ORCA の per-agent solve helper 抽出
- `ORCAAlgorithm.step()` から helper を利用するように整理
- 既存 ORCA test が壊れていないことの確認

## 非スコープ

- CNav ロジックの実装
- registry / CLI の更新

## 完了条件

- helper が `snapshot`, `agent_index`, `optimization_velocity` を受けて速度を返せる
- `ORCAAlgorithm.step()` が helper 経由で従来どおり動く
- 既存 ORCA 関連 test が通る

## 想定成果物

- `src/cmis_ca/algorithms/orca/` の helper
- ORCA 回帰 test または unit test の更新

## 作業メモ

- helper 名は ORCA 専用であることが分かるものにする
- CNav 以外からも再利用しやすいが、抽象化しすぎて `core/` へ移す必要はない

## 実施メモ

- `src/cmis_ca/algorithms/orca/agent_solver.py` に `compute_orca_velocity(...)` を追加した
- `ORCAAlgorithm.step()` は helper を使う形へ置き換えた
- arbitrary `optimization_velocity` を受け取れることと、既存 `step()` との一致を `tests/algorithms/test_orca.py` に追加した
