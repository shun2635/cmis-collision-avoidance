# Issue 0006: 共通コアの 2D 線形ソルバを移植する

- ステータス: completed
- 優先度: high
- 関連文書:
  - [docs/algorithms/orca.md](../algorithms/orca.md)
  - [docs/policies/source-file-policy.md](../policies/source-file-policy.md)
  - [src/cmis_ca/core/solver.py](../../src/cmis_ca/core/solver.py)
  - [src/cmis_ca/core/constraints.py](../../src/cmis_ca/core/constraints.py)
  - [external/RVO2/src/Agent.cc](../../external/RVO2/src/Agent.cc)

## 背景

ORCA の本実装へ進むには、半平面制約と最大速度円の中で速度を選ぶ 2 次元ソルバが必要である。  
upstream では `linearProgram1` `linearProgram2` `linearProgram3` が `Agent.cc` に埋め込まれているが、本リポジトリではこれを ORCA 固有ロジックから切り離し、共通コアへ置く設計にしている。

## 目的

`src/cmis_ca/core/solver.py` に、ORCA から利用できる共通 2D 線形ソルバを実装する。  
ただし、ORCA 固有名や ORCA 固有の意味付けは持ち込まず、`LineConstraint + speed limit` の汎用補助計算として整理する。

## スコープ

- upstream の `linearProgram1/2/3` の役割を整理する
- Python 側の中立 API として再設計する
- `core/solver.py` を本実装へ置き換える
- ソルバ単体テストを追加する
- 必要に応じて `constraints.py` の型を見直す

## 非スコープ

- ORCA の制約生成
- ORCA の 1 ステップ統合
- 上流との完全一致保証

## 完了条件

- `choose_preferred_velocity` が placeholder ではなくなる
- 半平面制約なし、単一制約、複数制約の基本ケースにテストがある
- 実装が ORCA 固有名に依存していない
- source file policy に反しない形で出典と改変方針を保てる

## 想定成果物

- `src/cmis_ca/core/solver.py`
- `tests/core/test_solver.py`
- 必要に応じた `src/cmis_ca/core/constraints.py`

## 作業メモ

- 直接コピーではなく、責務を再整理してから移植する
- upstream 由来のロジックを近く参照した場合は改変事実の記録方針に従う
- 将来の proxemic や CNav が別目的関数で利用できる形を保つ
- `solve_linear_constraints()` を中立 API とし、`choose_preferred_velocity()` はその薄い wrapper にする
- `solver.py` には upstream 参照と改変事実をヘッダコメントで残す
- 現段階では ORCA 制約生成が空配列のため、シミュレーション全体の挙動は従来の smoke case と同じ

## 依存関係

- [0004-core-reference-types-and-tests.md](0004-core-reference-types-and-tests.md): completed
