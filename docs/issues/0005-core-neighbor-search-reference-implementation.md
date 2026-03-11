# Issue 0005: 共通コアの近傍探索基準実装を作る

- ステータス: open
- 優先度: high
- 関連文書:
  - [docs/repository-architecture.md](../repository-architecture.md)
  - [docs/algorithms/orca.md](../algorithms/orca.md)
  - [src/cmis_ca/core/neighbor_search.py](../../src/cmis_ca/core/neighbor_search.py)
  - [src/cmis_ca/core/world.py](../../src/cmis_ca/core/world.py)

## 背景

現在の `NaiveNeighborSearch` は最小スケルトンであり、責務は正しいが、後続の ORCA 実装に使える基準実装としてはまだ不十分である。  
ORCA では近傍探索そのものは共通コアに置き、距離閾値や採用規則はアルゴリズム側に残す方針を取っているため、この境界をコードでも明確にする必要がある。

## 目的

`core/neighbor_search.py` に、研究室実装の基準となる参照近傍探索を整備する。  
まずは単純実装でよいが、入力と出力、探索対象、責務分担を後続実装に耐える形に固定する。

## スコープ

- `NeighborSet` の構造を見直す
- エージェント近傍探索を基準実装として整理する
- 障害物近傍の扱いを整理する
- ORCA 側がパラメータで問い合わせる形を維持する
- 近傍探索の単体テストを追加する

## 非スコープ

- kd-tree などの高速化
- ORCA 制約生成
- ソルバ実装
- シナリオローダ

## 完了条件

- `NeighborSearch` インターフェースが明確である
- 参照実装が複数エージェントシナリオで使える
- 距離順や `max_neighbors` の扱いにテストがある
- アルゴリズム固有の採用規則がコア側へ漏れていない

## 想定成果物

- `src/cmis_ca/core/neighbor_search.py`
- `tests/core/test_neighbor_search.py`

## 作業メモ

- 高速化前に正しさを優先する
- ORCA の `neighbor_dist` `max_neighbors` は ORCA 側パラメータとして残す
- 将来の proxemic でも再利用できるよう、探索機構と意味付けを分ける

## 依存関係

- [0004-core-reference-types-and-tests.md](0004-core-reference-types-and-tests.md): pending
