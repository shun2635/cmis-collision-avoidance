# Issue 0020: upstream Roadmap scenario を回帰比較へ追加する

- ステータス: open
- 優先度: medium
- 関連文書:
  - [docs/specifications/orca-parity-gap-review.md](../specifications/orca-parity-gap-review.md)
  - [docs/specifications/upstream-blocks-regression.md](../specifications/upstream-blocks-regression.md)
  - [external/RVO2/examples/Roadmap.cc](../../external/RVO2/examples/Roadmap.cc)

## 背景

現在の regression suite は `Circle` と `Blocks` までであり、global guidance を含む upstream example は未対応である。  
`Roadmap.cc` を扱えるようにすると、obstacle 周辺と goal 更新の差分をより厳しく見られる。

## 目的

`Roadmap.cc` を比較対象へ追加するか、または現構成では扱わない理由を docs に固定する。

## スコープ

- `Roadmap.cc` の依存要素を整理する
- 現 repo で表現可能かを判断する
- 可能なら regression helper を追加する
- 難しければ非対応理由と前提作業を docs 化する

## 非スコープ

- 汎用 roadmap planner の実装
- UI / 可視化

## 完了条件

- `Roadmap.cc` の扱い方針が docs にある
- ORCA regression suite の次の拡張候補が明確になっている
