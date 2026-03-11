# Issue 0001: Python 実装への初期移行

- ステータス: on_hold
- 優先度: high
- 関連文書:
  - [README.md](../../README.md)
  - [docs/design-overview.md](../design-overview.md)
  - [docs/api-draft.md](../api-draft.md)
  - [docs/source-file-policy.md](../source-file-policy.md)

## 背景

このリポジトリは初期構成として C++ スケルトンを置いているが、研究室内での学習コストと可読性を考えると、主実装言語は Python の方が適している。  
今後は Python を研究室独自実装の第一言語とし、`external/RVO2` は引き続き upstream 参照用コードとして保持する。

現状の README、設計文書、API 草案、ビルド導線は C++ 前提の記述を含んでいるため、方針転換を最初に明文化しないと以後の作業がぶれる。  
また、ORCA 単独ではなく proxemic や CNav も同一基盤で扱う前提が固まったため、Python 化だけを先行すると構造が再度ぶれる可能性がある。

## 保留理由

先に `共通コア + アルゴリズム切替` を前提にしたリポジトリ構造を確定する必要がある。  
詳細は [docs/repository-architecture.md](../repository-architecture.md) を参照する。

## 目的

Python 実装へ移行するための土台を整え、以後の ORCA 再実装を Python ベースで進められる状態にする。

## スコープ

- 実装言語を Python に切り替える方針を README と設計文書へ反映する
- Python 向けの標準的なリポジトリ構成を定義する
- 最小の Python パッケージ骨組みを追加する
- テストと実行例の最小導線を追加する
- 既存 C++ スケルトンの扱いを明確化する

## 非スコープ

- ORCA 制約生成の本実装
- 線形計画ソルバの本実装
- upstream との挙動比較
- 障害物 API の実装

## 完了条件

- README が Python 実装前提の説明へ更新されている
- 設計概要と API 草案が Python 前提の内容へ更新されている
- `src/` 以下に Python パッケージの最小骨組みがある
- `tests/` に Python ベースのスモークテストがある
- 実行例またはサンプルスクリプトが追加されている
- 既存 C++ スケルトンの扱いが文書上で明記されている

## 想定成果物

- `pyproject.toml`
- `src/cmis_ca/`
- `tests/`
- `examples/` または `scripts/`
- Python 方針へ更新された README / docs

## 作業メモ

- Python の対象バージョンは 3.11 以上を候補とする
- 型ヒントを前提にし、教育用途を意識して読みやすい実装を優先する
- テストは `pytest` を第一候補とする
- 整形・lint は `ruff` 系でまとめると保守しやすい
- C++ スケルトンは削除または `archive/` 退避のどちらかを選び、README と issue 上で理由を明記する

## 最初のタスク案

- README から CMake / C++ 固有記述を見直す
- `docs/design-overview.md` のディレクトリ方針を Python 構成へ更新する
- `docs/api-draft.md` を Python モジュール設計へ置き換える
- `pyproject.toml` を追加する
- `src/cmis_ca/` に `vector2.py` と `simulator.py` の最小スケルトンを追加する
- `tests/test_simulator_smoke.py` を追加する
