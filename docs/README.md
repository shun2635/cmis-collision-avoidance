# Docs

このディレクトリでは、研究室実装の設計、仕様、運用、アルゴリズム別文書、issue を責務ごとに分けて管理します。

## 構成

- `architecture/`
  - リポジトリ全体の方針、構造、API 設計
- `policies/`
  - 文書運用、由来コード管理、改変記録などの運用ルール
- `specifications/`
  - 実装済み内容を固定する仕様書
  - upstream 比較用の回帰条件とメトリクスもここに含める
- `algorithms/`
  - ORCA などのアルゴリズム別文書
- `issues/`
  - 実装・設計作業を管理する issue 文書

## 運用上の重要事項

- 実装を変更したら、同じ change で関連する仕様書と docs を更新する
- アルゴリズム固有の設計判断は `algorithms/` に記録する
- 共通構造や公開面が変わる場合は `architecture/` と `specifications/` も更新する

詳細な運用ルールは [policies/documentation-sync-policy.md](policies/documentation-sync-policy.md) を参照してください。
