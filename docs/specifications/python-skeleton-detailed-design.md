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
- 利用可能なアルゴリズムは `orca` と `cnav`
- 外部シナリオファイル読込は YAML / JSON の最小ローダまで実装済み
- ORCA 制約生成は基準実装まで完了
- 共通ソルバは `LineConstraint + speed limit` に対する基準実装まで完了
- upstream の agent / simulator 主要 parameter と clock を Python 側へ反映済み
- agent-agent ORCA line と solver の主要分岐を upstream ベースで監査済み
- neighbor search の主要 semantics を upstream ベースで監査済み
- C++ 側のコードは履歴的な足場または比較用として残置

## 3. コンポーネント構成と役割

| コンポーネント | 主なパス | 役割 |
| --- | --- | --- |
| CLI エントリ | `src/cmis_ca/cli/main.py` | `cmis-ca` の引数解析とコマンド実行 |
| 実行導線 | `src/cmis_ca/cli/run.py` | 内蔵デモシナリオを作成し、シミュレータを起動 |
| シナリオ入出力 | `src/cmis_ca/io/scenario_loader.py` | YAML / JSON から `Scenario` を復元 |
| アルゴリズム共通 IF | `src/cmis_ca/algorithms/base.py` | `step(snapshot)` を持つ共通 protocol |
| アルゴリズムレジストリ | `src/cmis_ca/algorithms/registry.py` | アルゴリズム名から実装を生成 |
| 共通 core | `src/cmis_ca/core/` | 幾何、状態、ワールド、近傍探索、制約表現、共通ソルバ、実行ループ |
| ORCA 固有実装 | `src/cmis_ca/algorithms/orca/` | ORCA 固有パラメータ、制約生成、ステップ判断 |
| CNav 固有実装 | `src/cmis_ca/algorithms/cnav/` | action set、intent cache、coordination evaluation、ORCA 再利用 |
| 補助スクリプト | `scripts/smoke_run.py` | 最小実行確認 |
| 回帰 helper | `src/cmis_ca/regression/` | upstream 由来 scenario の builder と metric 集計 |
| 回帰スクリプト | `scripts/compare_upstream_circle.py`, `scripts/compare_upstream_blocks.py`, `scripts/compare_upstream_roadmap.py` | upstream 条件の比較メトリクスを出力 |
| 可視化 | `src/cmis_ca/visualization/` | simulation history を viewer 向け trace へ変換し、PyQtGraph で再生する |
| テスト | `tests/` | core の単体テストとスモークテスト |
| upstream 参照 | `external/RVO2/` | 比較・参照用の外部コード |

### 3.1 `core/` の責務

| モジュール | 役割 |
| --- | --- |
| `geometry.py` | `Vector2` と基本的な 2D 幾何演算 |
| `agent.py` | `AgentProfile` と `AgentConfig` の定義、goal 情報、goal 自動更新の有無、upstream 寄り navigation parameter の保持 |
| `state.py` | `AgentState` `AgentCommand` `SimulationResult` の定義 |
| `scenario.py` | draft API 互換のため `Scenario` を再公開 |
| `world.py` | `Scenario` `ObstaclePath` `ObstacleVertex` `SnapshotAgent` `WorldSnapshot` の定義 |
| `neighbor_search.py` | `NeighborSearch` protocol、距離付き近傍結果、`NaiveNeighborSearch` |
| `constraints.py` | 中立な半平面制約 `LineConstraint` |
| `solver.py` | 制約付き速度選択の共通ソルバ |
| `simulation.py` | アルゴリズムに依存しないステップ更新ループ |
| `visualization/` | 可視化 trace model、trace builder、PyQtGraph viewer |

### 3.2 `algorithms/orca/` の責務

| モジュール | 役割 |
| --- | --- |
| `parameters.py` | `ORCAParameters` の定義 |
| `constraints.py` | 障害物由来制約とエージェント間制約の生成 |
| `agent_solver.py` | arbitrary optimization velocity に対する per-agent ORCA solve |
| `algorithm.py` | 近傍探索、制約生成、ソルバ呼び出しをまとめた ORCA 1 ステップ |

### 3.3 `algorithms/cnav/` の責務

| モジュール | 役割 |
| --- | --- |
| `parameters.py` | `CNavParameters` の定義 |
| `actions.py` | 論文既定の 8 action 生成 |
| `coordination.py` | constrained neighbor ranking、short-horizon action evaluation、reward 計算 |
| `algorithm.py` | intended velocity cache と ORCA 呼び出しをまとめた CNav 1 ステップ |

### 3.4 `neighbor_search.py` の現行仕様

- `NeighborSet` は `agent_neighbors` と `obstacle_neighbors` を持つ
- 各要素は `index` と `distance` を持つ距離付き結果である
- 互換性のため、`agent_indices` と `obstacle_indices` を property として残す
- `NaiveNeighborSearch` は `agent_index` を `WorldSnapshot.agents` の tuple 位置ではなく agent ID として解決する
- エージェント近傍は `neighbor_dist^2` を strict `<` で比較し、stable insertion で距離順を保つ
- `max_neighbors` 到達後は末尾距離で agent range を縮め、upstream `insertAgentNeighbor()` と同様に打ち切る
- 障害物近傍は `obstacle_range` を別引数で受け取れ、未指定時だけ `neighbor_dist` を使う
- 障害物近傍は directed edge の右側にある agent だけを対象にし、線分距離の strict `<` で採用する
- `neighbor_dist < 0`、`max_neighbors < 0`、`obstacle_range < 0` は `ValueError` とする

### 3.5 `solver.py` の現行仕様

- `solve_linear_constraints()` は半平面制約列と速度上限円に対する 2D ソルバである
- `choose_preferred_velocity()` は「希望速度に最も近い点」を選ぶ wrapper である
- upstream `RVO2` の `linearProgram1/2/3` を中立 API へ再構成した実装である
- parallel 判定の `EPSILON` は upstream と同じ `1e-5` を使う
- `protected_constraint_count` により、先頭 N 本の制約を固定した fallback projection を扱える
- `max_speed < 0` や不正な `protected_constraint_count` は `ValueError` とする

### 3.6 `algorithms/orca/constraints.py` の現行仕様

- `build_agent_constraints()` は upstream の agent-agent ORCA line 生成をベースに、エージェント間の reciprocal avoidance 制約を作る
- no-collision の cut-off circle / left leg / right leg と collision 分岐を upstream ベースで持つ
- `build_obstacle_constraints()` は linked obstacle topology の outgoing edge に対して、upstream `Agent.cc` の obstacle 分岐を移植した ORCA 制約を作る
- エージェント間制約は回避責任を `0.5` ずつ分担する
- 障害物制約は静的障害物として agent 側が全責任を負う
- どちらも `LineConstraint` を返し、ORCA 固有の意味付けは `algorithms/orca/` に閉じ込める

### 3.7 `ORCAAlgorithm.step()` の現行仕様

- `NaiveNeighborSearch`、ORCA 制約生成、共通 solver をまとめて 1 ステップへ統合している
- `AgentProfile` の `neighbor_dist`, `max_neighbors`, `time_horizon`, `time_horizon_obst` を既定値として参照する
  - 現在の共通既定値は `time_horizon = 0.3`, `time_horizon_obst = 0.3`
- `ORCAParameters` は algorithm 側の optional override として扱う
- obstacle range は `time_horizon_obst * max_speed + radius` として計算し、agent range と分離して近傍探索へ渡す
- obstacle 制約は先頭へまとめ、solver の `protected_constraint_count` に渡す
- `AgentCommand` へ変換された速度指令を返す

### 3.8 `CNavAlgorithm.step()` の現行仕様

- `goal_position` が全 agent に必要
- `update_every_step = false` のとき、action 更新周期は `global_time` と `action_update_interval` で判定する
- `update_every_step = true` のとき、毎 step action を再評価する
- step 開始時点の intended velocity cache を communication 内容として固定参照する
- `rank_constrained_neighbors()` で、`自分より goal に近い neighbor` を constraint 量で順位付けする
- `select_best_action()` で、論文既定 8 action を `T=2` の short-horizon simulation で評価する
- 選んだ intended velocity を `compute_orca_velocity()` に渡して最終速度を得る
- 現時点では communication 遅延や action set の一般化は未実装

### 3.9 `io/scenario_loader.py` の現行仕様

- `.yaml` `.yml` `.json` を受け付ける
- トップレベルは mapping のみを受け付ける
- `agents` は必須で、1 件以上必要
- `name` 未指定時はファイル名 stem を使う
- `obstacles` は省略可能で、正規 schema は `vertices` と任意 `closed` を受け付ける
- `stop_when_all_agents_reach_goals` は任意で、`true` のとき既定実行は全 agent が goal 到達するまで継続する
- loader は互換のため `start` `end` 2 点定義も受け付ける
- ベクトルは `[x, y]` または `{x: ..., y: ...}` の 2 形式を受け付ける
- `agent.profile` では `radius`, `max_speed`, `neighbor_dist`, `max_neighbors`, `time_horizon`, `time_horizon_obst` を受け付ける
- `agent.goal_position` は任意で、指定時は vector として読み込む
- `agent.preferred_speed` は任意で、既定値は `1.0`
- `agent.auto_update_preferred_velocity_from_goal` は現時点では scenario loader 非対応で、コード生成 scenario だけで使う
- `--steps` が CLI から与えられた場合は、読み込んだ scenario の停止条件より CLI の固定 step 指定を優先する

### 3.10 既存構成上の制約

- `algorithms/registry.py` に登録されているアルゴリズムは `orca` と `cnav`
- goal モデルは導入済みだが、更新規則は現在 `goal_position` への直線追従に限る
- CNav は `goal_position` を必須とするが、scenario loader 自体は依然として goal 省略を許す

## 4. 入力仕様、出力仕様、DB スキーマ対応

### 4.1 CLI 入力仕様

現時点で実装されているコマンドは `run` と `visualize` である。

| 引数 | 型 | 既定値 | 現在の仕様 |
| --- | --- | --- | --- |
| `--algorithm` | `str` | `orca` | `algorithms/registry.py` の登録名 (`orca`, `cnav`) から選択 |
| `--steps` | `int \| None` | `None` | 指定時のみ固定 step 数を上書き。未指定時は内蔵デモは `1000`、外部シナリオは file 記述の step / goal-stop 条件を使う |
| `--scenario` | `str \| None` | `None` | YAML / JSON シナリオを読み込む |
| `--fps` | `float` | `30.0` | `visualize` でのみ使用。viewer の再生速度 |

### 4.2 Python API 入力仕様

`Simulator` を動かすための主要入力は以下。

| 型 | 主なフィールド | 用途 |
| --- | --- | --- |
| `AgentProfile` | `radius`, `max_speed`, `neighbor_dist`, `max_neighbors`, `time_horizon`, `time_horizon_obst` | エージェントの固定属性と ORCA 既定 parameter |
| `AgentConfig` | `name`, `profile`, `initial_position`, `initial_velocity`, `preferred_velocity`, `goal_position`, `preferred_speed`, `auto_update_preferred_velocity_from_goal`, `preferred_velocity_perturbation_scale`, `preferred_velocity_perturbation_phase` | シナリオ投入時の初期設定 |
| `Scenario` | `agents`, `obstacles`, `time_step`, `steps`, `stop_when_all_agents_reach_goals`, `name` | シミュレーション入力全体 |
| `WorldSnapshot` | `step_index`, `global_time`, `time_step`, `agents`, `obstacles` | 1 ステップごとの読み取り専用入力 |
| `NeighborSet` | `agent_neighbors`, `obstacle_neighbors` | 距離付き近傍探索結果 |
| `VisualizationTrace` | `frames`, `obstacles`, `initial_positions`, `agent_names` | 可視化 viewer 向けの再生データ |

現在の入力バリデーション:

- `Scenario.time_step` は正数
- `Scenario.steps` は 0 以上
- `Scenario.stop_when_all_agents_reach_goals=True` のとき全 agent に `goal_position` が必要
- `Scenario.agents` は 1 体以上必須
- `AgentProfile.radius` は 0 以上
- `AgentProfile.max_speed` は 0 以上
- `AgentProfile.neighbor_dist` は 0 以上
- `AgentProfile.max_neighbors` は 0 以上
- `AgentProfile.time_horizon` は正数
- `AgentProfile.time_horizon_obst` は正数
- `ObstaclePath` は 2 点以上必須
- `ObstaclePath` は連続重複点を禁止
- `ObstaclePath` は `closed: true` のとき先頭点の末尾再掲を禁止
- `WorldSnapshot.global_time` は 0 以上
- `WorldSnapshot.agents` の `index` は一意
- `SnapshotAgent.goal_position` は algorithm によって必須になり得る
- `LineConstraint.direction` はゼロベクトルを禁止
- `AgentConfig.preferred_speed` は 0 以上
- `AgentConfig.preferred_velocity_perturbation_scale` は 0 以上
- シナリオファイルはトップレベル mapping 必須
- シナリオファイルの `agents` は非空 list 必須
- ベクトル項目は 2 要素 list または `x` `y` を持つ mapping 必須

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
| `scenarios/*.yaml` | 共通シナリオ定義 |
| `scenarios/upstream_circle.yaml` | upstream `Circle.cc` 由来の比較用シナリオ |
| `src/cmis_ca/regression/upstream_blocks.py` | upstream `Blocks.cc` 由来の code-generated scenario と metric |
| `src/cmis_ca/regression/upstream_roadmap.py` | upstream `Roadmap.cc` 由来の code-generated scenario、visibility graph、metric |
| `src/cmis_ca/visualization/pyqtgraph_viewer.py` | PyQtGraph viewer backend |
| `docs/algorithms/orca.md` | ORCA の責務境界と設計方針 |

現時点で `configs/` は未使用だが、`scenarios/` は CLI から利用する。

### 5.3 環境構築手順

```bash
poetry install
```

### 5.4 実行手順

最小 CLI 実行:

```bash
poetry run cmis-ca run --algorithm orca --steps 1
poetry run cmis-ca run --algorithm orca --scenario scenarios/head_on.yaml
poetry run cmis-ca run --algorithm cnav --scenario scenarios/cnav_queue.yaml
poetry run cmis-ca visualize --algorithm orca --scenario scenarios/head_on.yaml
poetry run cmis-ca visualize --algorithm cnav --scenario scenarios/cnav_queue.yaml
```

補助スクリプト:

```bash
poetry run python scripts/smoke_run.py
```

テスト実行:

```bash
poetry run pytest
```

## 6. 処理フロー、ロジック

### 6.1 現在の全体フロー

1. CLI で `cmis-ca run` を受け付ける
2. `algorithms/registry.py` から指定アルゴリズムを生成する
3. `--scenario` があれば `io/scenario_loader.py` が YAML / JSON を `Scenario` へ変換する
4. `--scenario` がなければ `cli/run.py` が内蔵の circle scenario を構築する
5. `Simulator` が `Scenario` から内部 `AgentState` を初期化する
6. `goal_position` を持ち、かつ `auto_update_preferred_velocity_from_goal=True` の agent がいれば、各 step の直前に `goal_position - current_position` から preferred velocity を更新する
7. `preferred_velocity_perturbation_scale > 0` の agent では、完全対称の停滞を避けるため deterministic perturbation を希望速度へ加える
8. `Simulator.snapshot()` が `global_time` を含む `WorldSnapshot` を作る
9. `ORCAAlgorithm.step()` が各 agent の `AgentProfile` を既定値として ORCA parameter を解決し、近傍探索、制約生成、速度選択を行う
10. `Simulator.step()` が返却された `AgentCommand` を速度制限付きで適用し、位置を更新する
11. `Simulator` は `global_time += time_step` を進める
12. `Simulator.run()` は、通常は固定 step 数で、`stop_when_all_agents_reach_goals=True` のときは goal 到達判定で停止しつつ履歴をまとめて `SimulationResult` を返す
13. `visualize` 導線では `visualization/trace_builder.py` が `Scenario` と `SimulationResult` を `VisualizationTrace` へ変換する
14. `visualization/pyqtgraph_viewer.py` が障害物、初期位置、現在位置、軌跡を再生表示する

### 6.5 upstream Circle 回帰の現行ロジック

1. `scenarios/upstream_circle.yaml` から 250 体の円周配置を読み込む
2. `external/RVO2/examples/Circle.cc` と同じく、各 agent の goal は `goal_position` に antipodal point として記述する
3. `Scenario.stop_when_all_agents_reach_goals=True` により、既定実行は upstream と同じく全 agent の goal 到達まで続く
4. `Simulator.refresh_preferred_velocities_from_goals()` が各 step の直前に `goal - current_position` から希望速度を再計算する
5. 目標ベクトル長が `preferred_speed` を超えるときは正規化して速度をそろえ、近傍なら goal までの残差ベクトルをそのまま使う
6. ORCA パラメータは各 agent の `profile.neighbor_dist=15`, `profile.max_neighbors=10`, `profile.time_horizon=10`, `profile.time_horizon_obst=10` を使う
7. 明示的に `steps` override を渡した場合だけ、goal 到達停止ではなく固定 step 実行へ切り替える
8. 実行後は平均半径、最短 pair 距離、重心ずれ、goal 距離、速度上限、antipodal 対称性を比較メトリクスとして集計する

### 6.6 upstream Blocks 回帰の現行ロジック

1. `src/cmis_ca/regression/upstream_blocks.py` が `external/RVO2/examples/Blocks.cc` に対応する `Scenario` を code-generated で構築する
2. 100 体の agent を 4 隅へ 25 体ずつ配置し、対角側 goal を与える
3. 4 個の polygon obstacle を upstream と同じ頂点順で配置する
4. `Blocks` では upstream と同様に helper 側が各 step の直前に goal 方向の unit vector と微小 perturbation から preferred velocity を更新する
5. 実行後は average goal distance、goal distance reduction、最短 pair 距離、重心ずれ、速度上限、central agent count を集計する

### 6.7 upstream Roadmap 回帰の現行ロジック

1. `src/cmis_ca/regression/upstream_roadmap.py` が `external/RVO2/examples/Roadmap.cc` に対応する `Scenario`、roadmap vertex、goal index を code-generated で構築する
2. obstacle は `Blocks` と同じ 4 polygon を使い、goal vertex 4 個と obstacle 周辺 vertex 16 個で visibility graph を作る
3. visibility graph は regression helper 内の segment intersection と clearance 判定を使って構築し、goal までの距離表は Dijkstra で求める
4. 実行時は static `goal_position` を使わず、各 step の直前に「可視な roadmap vertex のうち最短経路上で最も良いもの」へ向かう preferred velocity を更新する
5. upstream の微小 perturbation は deterministic seed で再現し、回帰の再現性を優先する
6. 実行後は goal distance reduction、最短 pair 距離、重心ずれ、速度上限、goal 到達数を集計する

### 6.2 ORCA 1 ステップの現状ロジック

1. `NaiveNeighborSearch` が agent range と obstacle range を分けて受け取り、strict 境界で近傍エージェントを列挙する
2. linked obstacle topology のうち `next_index` を持つ outgoing edge を対象に、directed edge の右側だけを obstacle 近傍候補にする
3. どちらも stable insertion で距離順を保ち、エージェント近傍のみ `max_neighbors` と更新済み range で制限する
4. `build_obstacle_constraints()` が static obstacle edge から ORCA 制約を生成する
5. `build_agent_constraints()` が近傍エージェントから reciprocal avoidance 制約を生成する
6. `solve_linear_constraints()` が obstacle 制約数を `protected_constraint_count` として受け取り、速度上限円付きの制約付き速度選択を行う
7. 現時点の built-in demo は 8 エージェントの circle なので、エージェント間制約が最初の step から発生し得る
8. `Simulator.step()` が `position += velocity * time_step` で更新する

したがって、現時点の ORCA 実装は「責務分離を保ったまま、近傍探索・制約生成・共通 solver まで統合した最小構成」である。

### 6.3 内蔵デモシナリオ

`cli/run.py` の `run_demo()` は、以下の built-in circle scenario を内蔵する。

- 名前: `circle-demo`
- エージェント数: 8
- 配置半径: `8.0`
- 各 agent の半径: `0.4`
- 各 agent の最大速度: `1.0`
- goal: antipodal point
- preferred speed: `1.0`
- `time_step`: `0.5`
- 既定 step 数: `1000`
- goal 追従時にごく小さい deterministic perturbation を加えて、8 体の完全対称停滞を避ける

このため、`run` と `visualize` の既定挙動でも複数 agent の相互作用を確認できる。

### 6.4 外部シナリオファイルの最小スキーマ

```yaml
name: head-on-demo
time_step: 0.5
steps: 2
stop_when_all_agents_reach_goals: false
agents:
  - name: left
    profile:
      radius: 0.5
      max_speed: 1.0
    initial_position: [-1.0, 0.0]
    initial_velocity: [0.0, 0.0]
    preferred_velocity: [1.0, 0.0]
obstacles:
  - closed: true
    vertices:
      - [0.3, -1.0]
      - [0.3, 1.0]
      - [0.7, 1.0]
      - [0.7, -1.0]
```

## 7. テスト方針

### 7.1 現在のテスト構成

| テストファイル | 対象 |
| --- | --- |
| `tests/algorithms/test_orca.py` | ORCA step の単独・head-on・障害物ケース |
| `tests/cli/test_main.py` | `--scenario` 読込と `--steps` 上書き |
| `tests/algorithms/test_orca_constraints.py` | agent-agent / obstacle 制約の非衝突・衝突・left/right leg・foreign leg ケース |
| `tests/core/test_agent.py` | `AgentProfile` の navigation parameter と ORCA override 解決 |
| `tests/core/test_geometry.py` | `Vector2` の演算、正規化、距離、例外 |
| `tests/core/test_neighbor_search.py` | 距離順、`max_neighbors`、障害物近傍、入力検証 |
| `tests/core/test_simulation.py` | goal ベースの preferred velocity 自動更新 |
| `tests/core/test_solver.py` | 半平面制約なし、単一制約、複数制約、parallel opposite line、protected fallback、入力検証 |
| `tests/core/test_state.py` | `AgentState` `AgentCommand` `SimulationResult` |
| `tests/core/test_world.py` | `Scenario` `ObstaclePath` `ObstacleVertex` `WorldSnapshot` `LineConstraint` |
| `tests/io/test_scenario_loader.py` | YAML / JSON ローダ、スキーマ違反 |
| `tests/regression/test_upstream_blocks.py` | upstream Blocks の条件と障害物付き進捗確認 |
| `tests/regression/test_upstream_circle.py` | upstream Circle の条件と対称性・内向き収束 |
| `tests/test_simulator_smoke.py` | ORCA skeleton の 1 エージェント前進確認 |

### 7.2 方針

- 共通 core の型と基本演算は unit test で固定する
- 実行導線は smoke test で最低限保証する
- placeholder 実装であっても、現在の振る舞いはテストで固定する
- テストランナーの標準は `pytest` とする
- ORCA 本実装が入ったら、制約生成とソルバを別テストで分離する
- 将来的には `tests/algorithms/` `tests/integration/` `tests/regression/` を拡充する

### 7.3 テスト更新ルール

実装の振る舞いを変えた場合は、仕様書だけでなく対応テストも同じ change で更新する。  
テストだけ、または docs だけが先行して乖離しないようにする。

## 8. 未実装事項

現時点の主要な未実装事項は以下。

- 複数アルゴリズム登録
- 回帰 suite のさらなる拡張
- 結果保存、可視化

これらが実装されたら、本書を更新して skeleton 状態の記述を置き換える。
