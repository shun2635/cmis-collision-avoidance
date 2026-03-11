# CMIS Collision Avoidance

研究室向けの collision avoidance 研究用リポジトリです。  
ORCA (Optimal Reciprocal Collision Avoidance) を軸に、将来的には proxemic や CNav などの関連アルゴリズムも同一基盤上で扱えるように設計します。  
upstream である `snape/RVO2` を参照しつつ、研究室内で読みやすく保守しやすい構成と日本語ドキュメントを備えた再実装を進めます。

## このリポジトリの位置付け

- この repo は `snape/RVO2` を参照または一部利用した研究室向け派生プロジェクトです。
- upstream URL: <https://github.com/snape/RVO2>
- upstream のライセンスは Apache License 2.0 です。
- この repo は upstream の公式版ではありません。
- third-party code の詳細は [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md) を参照してください。
- この repo 全体のライセンスは [LICENSE](LICENSE) を参照してください。
- `external/RVO2` は外部参照コードであり、研究室独自実装は `src/` 以下で管理します。
- upstream 由来コードには元のライセンス条件が適用されます。
- 改変を含むファイルでは、改変事実を記録する方針です。詳細は [docs/policies/source-file-policy.md](docs/policies/source-file-policy.md) を参照してください。

## 目的

- ORCA を中心とした collision avoidance アルゴリズム群を研究室内で理解しやすい形に再構成する
- 日本語の設計文書と仕様書を先に整備する
- upstream を参照しながら、共通コアを持つ研究室向け実装を段階的に構築する
- 実装変更と docs を常に同期させ、引き継ぎコストを下げる
- 将来的に upstream との比較で妥当性を確認する

## 現在の状態

現時点の正規実装は Python 側の `src/cmis_ca/` です。  
最小スケルトンとして、`core/` と `algorithms/orca/` の責務分離、Poetry 環境、CLI、スモークテスト、共通コアの基準型を整備しています。

現時点の実装制約:

- 利用可能なアルゴリズムは `orca` のみ
- `--scenario` による外部シナリオ読込は未実装
- ORCA 制約生成は placeholder 段階
- 共通ソルバは `LineConstraint + speed limit` の基準実装まで完了

実装済みの詳細仕様は [docs/specifications/python-skeleton-detailed-design.md](docs/specifications/python-skeleton-detailed-design.md) を参照してください。

## リポジトリ構成

```text
.
├── docs/
│   ├── architecture/       # 設計方針と構造設計
│   ├── policies/           # 運用ポリシー
│   ├── specifications/     # 実装同期用の仕様書
│   ├── algorithms/         # アルゴリズム別文書
│   └── issues/             # 作業単位の管理
├── external/RVO2/          # upstream 参照用コード
├── src/cmis_ca/            # Python 実装本体
├── scripts/                # 最小サンプル実行
├── scenarios/              # シナリオ定義
└── tests/                  # 自動テスト
```

## 実装方針

1. 新規 repo として管理する
2. upstream は `external/` に参照用として保持する
3. 先に設計文書を書く
4. 共通コアとアルゴリズム差分を分離した構造を採用する
5. 1 コマンドでアルゴリズムを切り替えられる CLI を用意する
6. まず ORCA を再実装し、その後に proxemic や CNav を追加する
7. 実装変更時は、同じ change で関連仕様書と docs を更新する
8. upstream 比較で妥当性を確認する

## 開発

Poetry 環境の作成:

```bash
poetry install
```

CLI の最小形:

```bash
poetry run cmis-ca run --algorithm orca --steps 1
```

最小スケルトンの実行確認:

```bash
poetry run python scripts/smoke_run.py
poetry run pytest
```

将来的には以下のような実行形へ広げます。

```bash
cmis-ca run --algorithm orca --scenario scenarios/circle.yaml
cmis-ca run --algorithm proxemic --scenario scenarios/circle.yaml
cmis-ca run --algorithm cnav --scenario scenarios/circle.yaml
```

## 移行メモ

既存の C++ スケルトンは、初期検討時の暫定実装として残しています。  
現時点で正規の実装対象は Python 側の `src/cmis_ca/` であり、`include/` `src/*.cpp` `examples/*.cpp` `tests/*.cpp` は比較用または履歴的な足場として扱います。

Poetry の仮想環境は [poetry.toml](poetry.toml) によりプロジェクト内 `.venv/` を使う前提です。

## 文書

- docs 一覧: [docs/README.md](docs/README.md)
- 設計概要: [docs/architecture/overview.md](docs/architecture/overview.md)
- リポジトリ構造: [docs/architecture/repository-structure.md](docs/architecture/repository-structure.md)
- API 設計: [docs/architecture/api.md](docs/architecture/api.md)
- 実装同期ポリシー: [docs/policies/documentation-sync-policy.md](docs/policies/documentation-sync-policy.md)
- 由来コードの取り扱い方針: [docs/policies/source-file-policy.md](docs/policies/source-file-policy.md)
- 現行実装の詳細設計書: [docs/specifications/python-skeleton-detailed-design.md](docs/specifications/python-skeleton-detailed-design.md)
