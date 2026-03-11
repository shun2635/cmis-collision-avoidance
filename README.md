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
- 改変を含むファイルでは、改変事実を記録する方針です。詳細は [docs/source-file-policy.md](docs/source-file-policy.md) を参照してください。

## 目的

- ORCA を中心とした collision avoidance アルゴリズム群を研究室内で理解しやすい形に再構成する
- 日本語の設計文書と API 文書を先に整備する
- upstream を参照しながら、共通コアを持つ研究室向け実装を段階的に構築する
- 将来的に upstream との比較で妥当性を確認する

## 現在の状態

この段階では、以下を先行して整備しています。

- ライセンスと third-party notice の整理
- 設計文書と API 草案
- 将来の多アルゴリズム対応を見据えた設計文書
- 暫定的な実装スケルトン

現時点の実装は完成版ではなく、後続の Python ベース再構成に向けた準備段階です。  
Python の最小スケルトンは `src/cmis_ca/` に追加済みで、`core/` と `algorithms/orca/` の責務分離を先に反映しています。

## 想定リポジトリ構成

```text
.
├── docs/                   # 設計文書、API 草案、運用方針
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
7. upstream 比較で妥当性を確認する

設計の詳細は [docs/design-overview.md](docs/design-overview.md) を参照してください。

リポジトリ構造の詳細は [docs/repository-architecture.md](docs/repository-architecture.md) を参照してください。

## 開発

最小スケルトンの実行確認:

```bash
PYTHONPATH=src python3 scripts/smoke_run.py
PYTHONPATH=src python3 -m unittest discover -s tests -p "test_*.py"
```

CLI の最小形:

```bash
PYTHONPATH=src python3 -m cmis_ca.cli.main run --algorithm orca --steps 1
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

## 文書

- 設計概要: [docs/design-overview.md](docs/design-overview.md)
- リポジトリ構造: [docs/repository-architecture.md](docs/repository-architecture.md)
- API 草案: [docs/api-draft.md](docs/api-draft.md)
- 由来コードの取り扱い方針: [docs/source-file-policy.md](docs/source-file-policy.md)
