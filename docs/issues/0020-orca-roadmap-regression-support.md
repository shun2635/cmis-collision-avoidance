# Issue 0020: upstream Roadmap scenario を回帰比較へ追加する

- ステータス: completed
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

## 実施メモ

- `src/cmis_ca/regression/upstream_roadmap.py` を追加し、scenario、visibility graph、Dijkstra 距離表、preferred velocity 更新を regression helper に閉じ込めた
- `scripts/compare_upstream_roadmap.py` と `tests/regression/test_upstream_roadmap.py` を追加した
- public な roadmap planner は追加せず、regression-local な visibility helper と deterministic perturbation で比較条件を固定した
- 結果を [../specifications/upstream-roadmap-regression.md](../specifications/upstream-roadmap-regression.md) に記録した
