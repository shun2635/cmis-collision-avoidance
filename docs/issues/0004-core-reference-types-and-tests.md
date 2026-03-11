# Issue 0004: 共通コアの基準型と基本テストを固める

- ステータス: completed
- 優先度: high
- 関連文書:
  - [docs/architecture/repository-structure.md](../architecture/repository-structure.md)
  - [docs/algorithms/orca.md](../algorithms/orca.md)
  - [docs/architecture/api.md](../architecture/api.md)
  - [src/cmis_ca/core/geometry.py](../../src/cmis_ca/core/geometry.py)
  - [src/cmis_ca/core/state.py](../../src/cmis_ca/core/state.py)
  - [src/cmis_ca/core/world.py](../../src/cmis_ca/core/world.py)
  - [src/cmis_ca/core/constraints.py](../../src/cmis_ca/core/constraints.py)

## 背景

Python スケルトンは追加できたが、`core/` 配下の型はまだ最小の置き場を作った段階であり、後続実装の前提として十分に固まっていない。  
ORCA 実装へ進む前に、ベクトル演算、状態型、シナリオ型、制約型といった共通コアの基準型を安定化し、最低限の単体テストを整備しておく必要がある。

## 目的

`src/cmis_ca/core/` に置く基準型を、後続 issue が安心して参照できる状態にする。  
特に `orca.md` で共通コアと分類した要素について、責務と振る舞いを固定する。

## スコープ

- `geometry.py` の基本演算を見直す
- `state.py` の状態型と結果型を整理する
- `world.py` の `Scenario` `WorldSnapshot` 系を整理する
- `constraints.py` の中立的な制約型を整理する
- それぞれに対応する単体テストを追加する

## 非スコープ

- ORCA の制約生成
- 線形計画ソルバの本実装
- シナリオローダ
- upstream 比較

## 完了条件

- 共通コア型の責務がコード上で読み取れる
- `Vector2` の加減乗、正規化、長さ制限などにテストがある
- `Scenario` `WorldSnapshot` `LineConstraint` に最低限の生成テストがある
- 後続 issue が import 先として使う型が安定している

## 想定成果物

- `src/cmis_ca/core/geometry.py`
- `src/cmis_ca/core/state.py`
- `src/cmis_ca/core/world.py`
- `src/cmis_ca/core/constraints.py`
- `tests/core/`

## 作業メモ

- ORCA 固有名をコア型へ混ぜない
- `LineConstraint` は中立名を維持する
- テストは実装を固定するための仕様書として書く

## 依存関係

- [0001-python-bootstrap.md](0001-python-bootstrap.md): completed
- [0002-orca-boundary-definition.md](0002-orca-boundary-definition.md): completed
