# CMIS Collision Avoidance

研究室向けの ORCA (Optimal Reciprocal Collision Avoidance) 派生プロジェクトです。  
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

- ORCA の実装を研究室内で理解しやすい形に再構成する
- 日本語の設計文書と API 文書を先に整備する
- upstream を参照しながら、研究室向けの最小実装を段階的に構築する
- 将来的に upstream との比較で妥当性を確認する

## 現在の状態

この初期構成では、以下を先行して整備しています。

- ライセンスと third-party notice の整理
- 設計文書と API 草案
- 研究室独自実装のための `src/` `include/` `tests/` の骨組み
- CMake ベースの最小ビルド構成

現時点の `src/` 実装は ORCA の完成版ではなく、後続の再実装に向けた最小スケルトンです。

## リポジトリ構成

```text
.
├── external/RVO2/          # upstream 参照用コード
├── docs/                   # 設計文書、API 草案、運用方針
├── include/cmis/orca/      # 研究室向け公開ヘッダ
├── src/                    # 研究室独自実装
├── examples/               # 最小利用例
└── tests/                  # スモークテスト
```

## 実装方針

1. 新規 repo として管理する
2. upstream は `external/` に参照用として保持する
3. 先に設計文書を書く
4. 研究室向け API を設計する
5. 最小 ORCA を再実装する
6. upstream 比較で妥当性を確認する

設計の詳細は [docs/design-overview.md](docs/design-overview.md) を参照してください。

## ビルド

```bash
cmake -S . -B build
cmake --build build
ctest --test-dir build
```

## 文書

- 設計概要: [docs/design-overview.md](docs/design-overview.md)
- API 草案: [docs/api-draft.md](docs/api-draft.md)
- 由来コードの取り扱い方針: [docs/source-file-policy.md](docs/source-file-policy.md)
