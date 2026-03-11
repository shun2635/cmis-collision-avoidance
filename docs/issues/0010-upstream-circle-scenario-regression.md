# Issue 0010: upstream Circle シナリオで比較用回帰基盤を作る

- ステータス: open
- 優先度: medium
- 関連文書:
  - [docs/design-overview.md](../design-overview.md)
  - [README.md](../../README.md)
  - [external/RVO2/examples/Circle.cc](../../external/RVO2/examples/Circle.cc)
  - [docs/algorithms/orca.md](../algorithms/orca.md)

## 背景

このリポジトリの長期目標には upstream 比較による妥当性確認が含まれている。  
最初の比較対象としては、upstream にある `Circle` シナリオが単純で分かりやすく、研究室向けの回帰基盤を作る題材としてちょうどよい。

## 目的

upstream の `Circle` シナリオに相当する条件を Python 側へ用意し、将来の ORCA 実装を比較・回帰確認できる土台を作る。

## スコープ

- upstream `Circle` シナリオの条件を読み解く
- Python 側に対応シナリオを作る
- 基本メトリクスまたは比較観点を定める
- 回帰テストまたは比較スクリプトの土台を作る

## 非スコープ

- upstream 完全一致の保証
- 大規模ベンチマーク
- proxemic / CNav 比較

## 完了条件

- `Circle` 相当シナリオが Python 側で再現できる
- 比較観点が文書またはテストで定義されている
- 将来の ORCA 実装差分を確認できる入口がある

## 想定成果物

- `scenarios/` 配下の比較用シナリオ
- `tests/regression/` または `scripts/compare_*`
- 必要に応じた比較メモ文書

## 作業メモ

- 最初は定性的比較でもよい
- 位置や速度の完全一致ではなく、衝突回避、対称性、収束傾向なども比較観点になり得る
- upstream 由来の条件を参照した場合は出典が追えるようにしておく

## 依存関係

- [0008-orca-step-integration.md](0008-orca-step-integration.md): pending
- [0009-scenario-schema-and-loader.md](0009-scenario-schema-and-loader.md): pending
