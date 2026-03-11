# Python 実装の詳細設計書

## 1. 目的と対象範囲

この文書は、`src/cmis_ca/` にある現行 Python 実装の詳細設計を記録する。  
対象は 2026-03-11 時点で実装済みの skeleton であり、将来案だけではなく現在の実装事実を固定する。

本書の対象:

- コンポーネント構成と責務
- 入出力仕様
- 現時点での永続化や DB スキーマ対応状況
- 環境構築、設定ファイル、実行手順
- 処理フローと主要ロジック
- テスト方針

## 2. 実装ステータス

現時点の実装ステータスは以下。

- 正規実装言語は Python
- 実装本体は `src/cmis_ca/`
- 利用可能なアルゴリズムは `orca` のみ
- 外部シナリオファイル読込は未実装
- ORCA 制約生成は placeholder
- 共通ソルバは placeholder
- C++ 側のコードは履歴的な足場または比較用として残置

## 3. コンポーネント構成と役割

| コンポーネント | 主なパス | 役割 |
| --- | --- | --- |
| CLI エントリ | `src/cmis_ca/cli/main.py` | `cmis-ca` の引数解析とコマンド実行 |
| 実行導線 | `src/cmis_ca/cli/run.py` | 内蔵デモシナリオを作成し、シミュレータを起動 |
| アルゴリズム共通 IF | `src/cmis_ca/algorithms/base.py` | `step(snapshot)` を持つ共通 protocol |
| アルゴリズムレジストリ | `src/cmis_ca/algorithms/registry.py` | アルゴリズム名から実装を生成 |
| 共通 core | `src/cmis_ca/core/` | 幾何、状態、ワールド、近傍探索、制約表現、共通ソルバ、実行ループ |
| ORCA 固有実装 | `src/cmis_ca/algorithms/orca/` | ORCA 固有パラメータ、制約生成、ステップ判断 |
| 補助スクリプト | `scripts/smoke_run.py` | 最小実行確認 |
| テスト | `tests/` | core の単体テストとスモークテスト |
| upstream 参照 | `external/RVO2/` | 比較・参照用の外部コード |

### 3.1 `core/` の責務

| モジュール | 役割 |
| --- | --- |
| `geometry.py` | `Vector2` と基本的な 2D 幾何演算 |
| `agent.py` | `AgentProfile` と `AgentConfig` の定義 |
| `state.py` | `AgentState` `AgentCommand` `SimulationResult` の定義 |
| `scenario.py` | draft API 互換のため `Scenario` を再公開 |
| `world.py` | `Scenario` `ObstacleSegment` `SnapshotAgent` `WorldSnapshot` の定義 |
| `neighbor_search.py` | `NeighborSearch` protocol と `NaiveNeighborSearch` |
| `constraints.py` | 中立な半平面制約 `LineConstraint` |
| `solver.py` | 制約付き速度選択の共通窓口 |
| `simulation.py` | アルゴリズムに依存しないステップ更新ループ |

### 3.2 `algorithms/orca/` の責務

| モジュール | 役割 |
| --- | --- |
| `parameters.py` | `ORCAParameters` の定義 |
| `constraints.py` | 障害物由来制約とエージェント間制約の生成窓口 |
| `algorithm.py` | 近傍探索、制約生成、ソルバ呼び出しをまとめた ORCA 1 ステップ |

### 3.3 既存構成上の制約

- `core/solver.py` は現在 `preferred_velocity` を最大速度で clamp するだけで、制約を評価していない
- `algorithms/orca/constraints.py` は現在空制約列を返す
- `cli/main.py` の `--scenario` は予約済みだが、指定するとエラーで終了する
- `algorithms/registry.py` に登録されているアルゴリズムは `orca` のみ

## 4. 入力仕様、出力仕様、DB スキーマ対応

### 4.1 CLI 入力仕様

現時点で実装されているコマンドは `run` のみ。

| 引数 | 型 | 既定値 | 現在の仕様 |
| --- | --- | --- | --- |
| `--algorithm` | `str` | `orca` | `algorithms/registry.py` の登録名から選択 |
| `--steps` | `int` | `1` | 内蔵デモシナリオのステップ数に反映 |
| `--scenario` | `str \| None` | `None` | 未実装。指定すると parser error |

### 4.2 Python API 入力仕様

`Simulator` を動かすための主要入力は以下。

| 型 | 主なフィールド | 用途 |
| --- | --- | --- |
| `AgentProfile` | `radius`, `max_speed` | エージェントの固定物理属性 |
| `AgentConfig` | `name`, `profile`, `initial_position`, `initial_velocity`, `preferred_velocity` | シナリオ投入時の初期設定 |
| `Scenario` | `agents`, `obstacles`, `time_step`, `steps`, `name` | シミュレーション入力全体 |
| `WorldSnapshot` | `step_index`, `time_step`, `agents`, `obstacles` | 1 ステップごとの読み取り専用入力 |

現在の入力バリデーション:

- `Scenario.time_step` は正数
- `Scenario.steps` は 0 以上
- `Scenario.agents` は 1 体以上必須
- `ObstacleSegment` は長さ 0 を禁止
- `WorldSnapshot.agents` の `index` は一意
- `LineConstraint.direction` はゼロベクトルを禁止

### 4.3 出力仕様

#### CLI 出力

`cmis-ca run` は、最終状態の各エージェントについて以下を標準出力へ 1 行ずつ表示する。

```text
agent=<index> position=(<x>, <y>) velocity=(<vx>, <vy>)
```

#### Python API 出力

`Simulator.run()` は `SimulationResult` を返す。

| フィールド | 型 | 内容 |
| --- | --- | --- |
| `algorithm` | `str` | 使用アルゴリズム名 |
| `final_states` | `tuple[AgentState, ...]` | 最終ステップ後の状態 |
| `history` | `tuple[tuple[AgentState, ...], ...]` | 初期状態を含む全履歴 |
| `num_steps` | `int` | `history` から算出される実行ステップ数 |

### 4.4 DB スキーマ対応

現時点では DB や永続化レイヤは実装していない。  
そのため、DB スキーマ対応は `該当なし` とする。

補足:

- シナリオ YAML/JSON ローダは将来実装予定だが未着手
- 結果のファイル出力、ログ保存、メトリクス保存も未実装
- 永続化が入る場合は、この節にスキーマ対応表を追加する

## 5. 環境構築、設定ファイル、実行手順

### 5.1 前提環境

- Python `>=3.9,<4.0`
- Poetry

バージョン制約は `pyproject.toml` で管理する。

### 5.2 設定ファイル

| ファイル | 役割 |
| --- | --- |
| `pyproject.toml` | Poetry 設定、依存関係、スクリプト登録、pytest/ruff 設定 |
| `poetry.toml` | Poetry 仮想環境を `.venv/` に作る設定 |
| `docs/algorithms/orca.md` | ORCA の責務境界と設計方針 |

現時点では `configs/` や外部シナリオ定義ファイルは未使用である。

### 5.3 環境構築手順

```bash
poetry install
```

### 5.4 実行手順

最小 CLI 実行:

```bash
poetry run cmis-ca run --algorithm orca --steps 1
```

補助スクリプト:

```bash
poetry run python scripts/smoke_run.py
```

テスト実行:

```bash
poetry run python -m unittest discover -s tests -p "test_*.py"
```

## 6. 処理フロー、ロジック

### 6.1 現在の全体フロー

1. CLI で `cmis-ca run` を受け付ける
2. `algorithms/registry.py` から指定アルゴリズムを生成する
3. `cli/run.py` が内蔵の 1 エージェントシナリオを構築する
4. `Simulator` が `Scenario` から内部 `AgentState` を初期化する
5. 各ステップで `Simulator.snapshot()` が `WorldSnapshot` を作る
6. `ORCAAlgorithm.step()` が近傍探索、制約生成、速度選択を行う
7. `Simulator.step()` が返却された `AgentCommand` を速度制限付きで適用し、位置を更新する
8. `Simulator.run()` が履歴をまとめて `SimulationResult` を返す

### 6.2 ORCA 1 ステップの現状ロジック

1. `NaiveNeighborSearch` が距離ベースで近傍エージェントを列挙する
2. 障害物近傍は、現時点では全障害物 index をそのまま返す
3. `build_obstacle_constraints()` は現在空配列を返す
4. `build_agent_constraints()` も現在空配列を返す
5. `choose_preferred_velocity()` は制約を無視し、希望速度を `max_speed` で clamp する
6. `Simulator.step()` が `position += velocity * time_step` で更新する

したがって、現時点の ORCA skeleton は「ORCA の責務境界を保ったまま、希望速度追従だけを行う最小ループ」である。

### 6.3 内蔵デモシナリオ

`cli/run.py` の `run_demo()` は、以下の 1 エージェントシナリオを内蔵する。

- 名前: `minimal-demo`
- エージェント数: 1
- 半径: `0.4`
- 最大速度: `1.0`
- 初期位置: `(0.0, 0.0)`
- 初期速度: `(0.0, 0.0)`
- 希望速度: `(1.0, 0.0)`
- `time_step`: `0.5`

このため、制約が空の現在実装では 1 ステップ後に `x = 0.5` まで前進する。

## 7. テスト方針

### 7.1 現在のテスト構成

| テストファイル | 対象 |
| --- | --- |
| `tests/core/test_geometry.py` | `Vector2` の演算、正規化、距離、例外 |
| `tests/core/test_state.py` | `AgentState` `AgentCommand` `SimulationResult` |
| `tests/core/test_world.py` | `Scenario` `ObstacleSegment` `WorldSnapshot` `LineConstraint` |
| `tests/test_simulator_smoke.py` | ORCA skeleton の 1 エージェント前進確認 |

### 7.2 方針

- 共通 core の型と基本演算は unit test で固定する
- 実行導線は smoke test で最低限保証する
- placeholder 実装であっても、現在の振る舞いはテストで固定する
- ORCA 本実装が入ったら、制約生成とソルバを別テストで分離する
- 将来的には `tests/algorithms/` `tests/integration/` `tests/regression/` を拡充する

### 7.3 テスト更新ルール

実装の振る舞いを変えた場合は、仕様書だけでなく対応テストも同じ change で更新する。  
テストだけ、または docs だけが先行して乖離しないようにする。

## 8. 未実装事項

現時点の主要な未実装事項は以下。

- 外部シナリオローダ
- ORCA 制約生成の本実装
- 半平面 + 円制約を扱う共通 2D ソルバ
- 複数アルゴリズム登録
- 結果保存、可視化、メトリクス計算
- upstream 比較用の回帰基盤

これらが実装されたら、本書を更新して skeleton 状態の記述を置き換える。
