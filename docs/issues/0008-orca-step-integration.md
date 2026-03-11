# Issue 0008: ORCA の 1 ステップ処理を統合する

- ステータス: open
- 優先度: high
- 関連文書:
  - [docs/algorithms/orca.md](../algorithms/orca.md)
  - [src/cmis_ca/algorithms/orca/algorithm.py](../../src/cmis_ca/algorithms/orca/algorithm.py)
  - [src/cmis_ca/core/simulation.py](../../src/cmis_ca/core/simulation.py)

## 背景

ORCA の責務境界、共通ソルバ、近傍探索、制約生成が揃って初めて、`ORCAAlgorithm.step()` を placeholder から実際の意思決定へ置き換えられる。  
この issue は、それらをまとめて「最初の動く ORCA」に統合する段階である。

## 目的

`ORCAAlgorithm.step()` を本実装へ置き換え、最小シナリオで ORCA の 1 ステップが動く状態にする。

## スコープ

- `ORCAAlgorithm.step()` を本実装にする
- 近傍探索、制約生成、ソルバ呼び出しを統合する
- `Simulator` と整合する `AgentCommand` を返す
- ORCA のスモークテストを拡張する

## 非スコープ

- シナリオファイル読み込み
- upstream 比較
- proxemic / CNav 実装

## 完了条件

- `ORCAAlgorithm.step()` が制約を使って速度を返す
- 単純な 2 エージェントケースで衝突回避らしい挙動が確認できる
- 既存 smoke test を壊さない
- ORCA の統合テストが追加されている

## 想定成果物

- `src/cmis_ca/algorithms/orca/algorithm.py`
- `tests/algorithms/test_orca.py`
- 必要に応じた `tests/integration/`

## 作業メモ

- 最初は小さなシナリオでよい
- `Simulator` 本体へ ORCA 固有分岐を入れない
- デバッグしやすいよう、必要なら ORCA 制約数などの補助情報を後で返せる設計を検討する

## 依存関係

- [0007-orca-constraint-generation.md](0007-orca-constraint-generation.md): pending
