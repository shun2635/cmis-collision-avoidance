# Issue 0018: ORCA 再現度の残差を棚卸しして次段階へ進む判断材料を作る

- ステータス: completed
- 優先度: medium
- 関連文書:
  - [docs/architecture/orca-reproduction-roadmap.md](../architecture/orca-reproduction-roadmap.md)
  - [docs/specifications/python-skeleton-detailed-design.md](../specifications/python-skeleton-detailed-design.md)
  - [docs/specifications/upstream-circle-regression.md](../specifications/upstream-circle-regression.md)

## 背景

ORCA 完全再現を目指す過程では、どこまで upstream に追いついたかを定期的に棚卸ししないと、次に何をやるべきか曖昧になる。  
また、proxemic / cnav へ進める時点を判断するためにも、差分一覧が必要である。

## 目的

ORCA の残差を監査し、未解決項目と妥協点を一覧化して、次段階へ進める条件を docs に固定する。

## スコープ

- 実装差分の棚卸し
- docs 間の記述整合性確認
- 必要なら追加 issue を切る
- ORCA 優先フェーズ完了判定の基準を記録する

## 非スコープ

- proxemic / cnav 実装そのもの
- UI や可視化の拡張

## 完了条件

- ORCA の残差一覧が docs にある
- 「次に multi-algorithm へ進めるか」の判断材料が揃う
- 必要な follow-up issue が整理されている

## 想定成果物

- `docs/specifications/` または `docs/architecture/`
- `docs/issues/`

## 依存関係

- [0017-orca-upstream-regression-suite-expansion.md](0017-orca-upstream-regression-suite-expansion.md): completed

## 実施メモ

- ORCA の残差一覧を [../specifications/orca-parity-gap-review.md](../specifications/orca-parity-gap-review.md) に整理した
- 「まだ `proxemic` / `cnav` へ主軸を移さない」判断を docs に固定した
- follow-up issue として `0019` と `0020` を追加した
- parity 文書の古い記述を見直し、docs 間の整合性を取った
