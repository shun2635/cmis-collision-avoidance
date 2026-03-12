# 設計概要

## 背景

`snape/RVO2` は ORCA の参照実装として有用ですが、研究室内の学習用途や継続的な保守を考えると、以下の課題があります。

- コードリーディングの前提知識が高い
- 日本語の説明が不足している
- upstream の構成が、そのまま研究室の開発運用に適しているとは限らない

そのため本リポジトリでは、upstream を参照しながら、ORCA を軸に proxemic や CNav なども同一基盤で扱える研究室向け実装を段階的に構築します。  
ただし当面の優先順位は `ORCA の upstream 再現度を上げること` に置き、他アルゴリズムへの展開はその後とします。

## 基本原則

1. `external/RVO2` は参照専用とし、研究室独自の修正は原則として加えない
2. 実装より先に設計文書を整備する
3. 実装を変更したら、同じ change で関連 docs と仕様書を更新する
4. 研究室独自実装は Python パッケージ `src/cmis_ca/` に集約する
5. 共通コアとアルゴリズム固有実装を分離する
6. public API と CLI は、アルゴリズム切替を前提に設計する
7. 最終的には upstream 比較で妥当性を検証する

## 実装フェーズ

### Phase 0: 初期構成

- ライセンス表記の整理
- third-party notice の整備
- 設計文書、API 草案、変更記録方針の整備
- 構造案の明文化

### Phase 1: 共通コアの整備

- 2 次元ベクトル演算
- エージェント状態管理
- ワールド状態とシナリオ定義
- 近傍探索
- 時間更新
- 共通ソルバと制約表現

複数アルゴリズムで共有できる部分を先に固定する。

### Phase 2: 最小 ORCA 再実装

- ORCA 制約生成
- 線形計画による速度決定

まずは読みやすさを優先し、必要であれば近傍探索を単純実装から始める。

### Phase 3: ORCA 完全再現

- goal / preferred velocity モデルの一般化 `completed`
- obstacle topology の再設計 `completed`
- obstacle ORCA 制約の厳密移植 `completed`
- agent / solver の差分監査 `completed`
- neighbor semantics の差分監査
- upstream scenario を用いた回帰 suite の拡張 `completed`

### Phase 4: 関連アルゴリズムの追加

- proxemic の追加
- CNav の追加
- アルゴリズム差分の文書化

### Phase 5: 研究室向け API と比較基盤の具体化

- シナリオ記述
- シミュレーション設定
- エージェントの追加、更新、観測 API
- 比較実験や教材用途を意識したログ/可視化連携点
- upstream 比較と既知ケースに対する回帰テスト追加

## ディレクトリ方針

```text
docs/               設計、仕様、運用方針
external/RVO2/      upstream 参照コード
scenarios/          共通シナリオ
configs/            共通設定、アルゴリズム別設定
src/cmis_ca/        Python 実装本体
tests/              自動テスト
```

詳細は [repository-structure.md](repository-structure.md) を参照する。  
実装済みの現行仕様は [../specifications/python-skeleton-detailed-design.md](../specifications/python-skeleton-detailed-design.md) を参照する。

## upstream との関係

- upstream は ORCA 理解と比較検証の基準として使う
- 研究室実装は upstream のファイル構成に拘束されない
- 研究室実装は ORCA のみを前提とせず、多アルゴリズム比較基盤として構成する
- ただし直近の issue 優先順位は ORCA 完全再現を最優先とする
- upstream 由来コードを持ち込む場合は、出典と改変事実を記録する

## 現時点の実装について

現時点の正規実装は Python 側の `src/cmis_ca/` である。  
`core/` と `algorithms/orca/` の責務分離、Poetry 環境、最小 CLI、スモークテスト、共通コアの基準型を先行して整備している。

一方で、neighbor semantics の厳密監査と回帰 suite の追加拡張は未完了である。  
この未完了部分を含む現状仕様は [../specifications/python-skeleton-detailed-design.md](../specifications/python-skeleton-detailed-design.md) で管理し、優先順位は [orca-reproduction-roadmap.md](orca-reproduction-roadmap.md) に整理する。
