# Issue 0007: ORCA 制約生成を実装する

- ステータス: open
- 優先度: high
- 関連文書:
  - [docs/algorithms/orca.md](../algorithms/orca.md)
  - [src/cmis_ca/algorithms/orca/constraints.py](../../src/cmis_ca/algorithms/orca/constraints.py)
  - [src/cmis_ca/algorithms/orca/parameters.py](../../src/cmis_ca/algorithms/orca/parameters.py)
  - [external/RVO2/src/Agent.cc](../../external/RVO2/src/Agent.cc)

## 背景

`orca.md` で、障害物由来制約とエージェント間制約は ORCA 固有差分として `algorithms/orca/` に閉じ込める方針を決めた。  
現在はその置き場だけがあり、実際の制約生成は未実装である。

## 目的

ORCA の 1 ステップに必要な制約生成を Python 側へ移植し、`constraints.py` を placeholder から実装へ置き換える。

## スコープ

- 障害物 ORCA 制約生成を実装する
- エージェント間 ORCA 制約生成を実装する
- ORCA パラメータとの結び付きを整理する
- ORCA 制約生成の単体テストを追加する

## 非スコープ

- ソルバの本実装
- `ORCAAlgorithm.step()` の統合
- upstream 比較

## 完了条件

- `build_obstacle_constraints()` が本実装になっている
- `build_agent_constraints()` が本実装になっている
- 非衝突ケースと衝突ケースの基本テストがある
- ORCA 固有ロジックが `core/` に漏れていない

## 想定成果物

- `src/cmis_ca/algorithms/orca/constraints.py`
- `tests/algorithms/test_orca_constraints.py`

## 作業メモ

- upstream の場合分けは研究室向けに責務で再整理する
- 制約型そのものは `core/constraints.py` を使う
- 実装途中で大きく膨らむなら、障害物制約とエージェント制約を別 issue に再分割してよい

## 依存関係

- [0005-core-neighbor-search-reference-implementation.md](0005-core-neighbor-search-reference-implementation.md): pending
- [0006-core-linear-solver-port.md](0006-core-linear-solver-port.md): pending
