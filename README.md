# CMIS Collision Avoidance

研究室向けの collision avoidance 研究用リポジトリです。  
ORCA (Optimal Reciprocal Collision Avoidance) を軸に、将来的には proxemic や CNav などの関連アルゴリズムも同一基盤上で扱えるように設計します。  
ただし直近の実装優先順位は `ORCA の upstream 再現度向上` に置き、proxemic / CNav は後回しにします。  
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
- 当面は ORCA 完全再現に必要な論点から順に詰める

## 現在の状態

現時点の正規実装は Python 側の `src/cmis_ca/` です。  
最小スケルトンとして、`core/` と `algorithms/orca/` の責務分離、Poetry 環境、CLI、スモークテスト、共通コアの基準型を整備しています。

現時点の実装制約:

- 利用可能なアルゴリズムは `orca` と `cnav`
- `--scenario` による YAML / JSON シナリオ読込に対応
- goal / preferred velocity の scenario 記述と step 前自動更新に対応
- scenario ごとの停止条件として `fixed steps` と `all goals reached` に対応
- upstream 寄りの per-agent ORCA defaults と `global_time` を Python 側へ反映済み
- ORCA 制約生成は基準実装まで完了
- 共通ソルバは `LineConstraint + speed limit` の基準実装まで完了
- 障害物 topology は upstream 寄りの linked vertex モデルへ移行済み
- 障害物 ORCA 制約は、upstream `Agent.cc` の obstacle 分岐をベースに移植済み
- agent-agent ORCA 制約と solver の主要分岐は upstream ベースで監査済み
- neighbor search は upstream ベースの range / boundary / insertion semantics に追従済み
- upstream `Circle` と `Blocks` 条件に基づく回帰シナリオと比較メトリクスの土台を追加済み
- upstream `Roadmap` 条件に基づく visibility-guided 回帰 helper を追加済み
- CNav の初期実装を追加し、論文既定 action set、intent cache、short-horizon action evaluation を ORCA 上へ統合済み

補足:

- `orca` は upstream 再現を主目的とする基準実装
- `cnav` は初期導入段階であり、比較ベンチマークや広い論文 scenario 再現は未実装

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
6. まず ORCA を upstream に近い形まで再実装し、その後に proxemic や CNav を追加する
7. 実装変更時は、同じ change で関連仕様書と docs を更新する
8. upstream 比較で妥当性を確認する

## 開発

Poetry 環境の作成:

```bash
poetry install
```

可視化 viewer 依存の `pyqtgraph` / `PySide6` / `numpy` は Python `>=3.9,<3.15` の範囲で解決されます。  
`numpy` は Python バージョンごとに互換のある系列へ固定しています。  
Python `>=3.15` では数値実行は維持し、`visualize` 導線は未対応です。

CLI の最小形:

```bash
poetry run cmis-ca run --algorithm orca --steps 1
poetry run cmis-ca run --algorithm orca --scenario scenarios/head_on.yaml
poetry run cmis-ca run --algorithm cnav --scenario scenarios/cnav_queue.yaml
poetry run cmis-ca visualize --algorithm orca --scenario scenarios/head_on.yaml
poetry run cmis-ca visualize --algorithm cnav --scenario scenarios/cnav_queue.yaml
```

`--scenario` を省略した場合の既定は、軽量な built-in `circle-demo` です。  
既定 step 数は `100` です。
この demo は 8 体の完全対称配置で中央停滞しないよう、goal 追従にごく小さい deterministic perturbation を加えています。

最小スケルトンの実行確認:

```bash
poetry run python scripts/smoke_run.py
poetry run pytest
```

現在利用できる実行形の例:

```bash
cmis-ca run --algorithm orca --scenario scenarios/head_on.yaml
cmis-ca run --algorithm cnav --scenario scenarios/cnav_queue.yaml
```

現時点で同梱しているシナリオ例:

- `scenarios/head_on.yaml`: 2 エージェントの正面衝突回避
- `scenarios/cnav_queue.yaml`: 同一 goal へ向かう 2 エージェントの CNav 用最小ケース
- `scenarios/obstacle_demo.yaml`: 単一障害物付きの最小ケース
- `scenarios/upstream_circle.yaml`: `external/RVO2/examples/Circle.cc` に基づく 250 体の比較用シナリオ。`steps: 0` と `stop_when_all_agents_reach_goals: true` により、既定では goal 到達まで回す

upstream 由来の regression helper:

- `scripts/compare_upstream_circle.py`: Circle の比較 metric を出力
- `scripts/compare_upstream_blocks.py`: Blocks の比較 metric を出力
- `scripts/compare_upstream_roadmap.py`: Roadmap の比較 metric を出力
- `Blocks` と `Roadmap` は repetitive な 100 体配置や roadmap graph を含むため、現時点では YAML ではなく `src/cmis_ca/regression/` の code-generated setup として保持する

## 移行メモ

既存の C++ スケルトンは、初期検討時の暫定実装として残しています。  
現時点で正規の実装対象は Python 側の `src/cmis_ca/` であり、`include/` `src/*.cpp` `examples/*.cpp` `tests/*.cpp` は比較用または履歴的な足場として扱います。

Poetry の仮想環境は [poetry.toml](poetry.toml) によりプロジェクト内 `.venv/` を使う前提です。

## 文書

- docs 一覧: [docs/README.md](docs/README.md)
- 設計概要: [docs/architecture/overview.md](docs/architecture/overview.md)
- ORCA 完全再現ロードマップ: [docs/architecture/orca-reproduction-roadmap.md](docs/architecture/orca-reproduction-roadmap.md)
- リポジトリ構造: [docs/architecture/repository-structure.md](docs/architecture/repository-structure.md)
- API 設計: [docs/architecture/api.md](docs/architecture/api.md)
- 可視化基盤設計: [docs/architecture/visualization-design.md](docs/architecture/visualization-design.md)
- 実装同期ポリシー: [docs/policies/documentation-sync-policy.md](docs/policies/documentation-sync-policy.md)
- 由来コードの取り扱い方針: [docs/policies/source-file-policy.md](docs/policies/source-file-policy.md)
- 現行実装の詳細設計書: [docs/specifications/python-skeleton-detailed-design.md](docs/specifications/python-skeleton-detailed-design.md)
- PyQtGraph 可視化基盤: [docs/specifications/pyqtgraph-visualization.md](docs/specifications/pyqtgraph-visualization.md)
- ORCA agent / simulator parity 監査: [docs/specifications/orca-agent-simulator-parity.md](docs/specifications/orca-agent-simulator-parity.md)
- ORCA neighbor search parity: [docs/specifications/orca-neighbor-search-parity.md](docs/specifications/orca-neighbor-search-parity.md)
- ORCA agent / solver parity: [docs/specifications/orca-agent-solver-parity.md](docs/specifications/orca-agent-solver-parity.md)
- ORCA obstacle topology: [docs/specifications/orca-obstacle-topology.md](docs/specifications/orca-obstacle-topology.md)
- ORCA parity gap review: [docs/specifications/orca-parity-gap-review.md](docs/specifications/orca-parity-gap-review.md)
- upstream Blocks 回帰基盤: [docs/specifications/upstream-blocks-regression.md](docs/specifications/upstream-blocks-regression.md)
- upstream Circle 回帰基盤: [docs/specifications/upstream-circle-regression.md](docs/specifications/upstream-circle-regression.md)
- upstream Roadmap 回帰基盤: [docs/specifications/upstream-roadmap-regression.md](docs/specifications/upstream-roadmap-regression.md)
- CNav アルゴリズム整理: [docs/algorithms/cnav.md](docs/algorithms/cnav.md)
- CNav 初期実装仕様: [docs/specifications/cnav-initial-implementation.md](docs/specifications/cnav-initial-implementation.md)
