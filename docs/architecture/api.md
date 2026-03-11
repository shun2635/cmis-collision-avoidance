# API 設計

## 目的

upstream の概念を踏まえつつ、研究室内で扱いやすい Python ベースの最小 API を定義する。  
初期段階では、教材・比較実験・将来のアルゴリズム追加に耐える単純な構成を優先する。

## 公開面

### Python パッケージ

`cmis_ca`

### CLI

`cmis-ca`

### 現在の CLI

```bash
poetry run cmis-ca run --algorithm orca --steps 1
```

現在は `run` サブコマンドのみ実装しており、`--scenario` による外部ファイル読込は未実装である。  
将来的な到達形は以下を想定する。

```bash
cmis-ca run --algorithm orca --scenario scenarios/circle.yaml
cmis-ca run --algorithm proxemic --scenario scenarios/circle.yaml
cmis-ca run --algorithm cnav --scenario scenarios/circle.yaml
```

## 基本データ型

- `Vector2`
  - 2 次元ベクトル
- `Agent`
  - `AgentConfig` の互換 alias
- `AgentConfig`
  - 半径、最大速度、初期位置、初期速度、希望速度などの設定
- `AgentState`
  - 現在位置、現在速度、希望速度
- `Scenario`
  - 初期配置、障害物、タイムステップ、総ステップ数
- `WorldSnapshot`
  - 1 ステップ分の読み取り専用ワールド状態
- `SimulationResult`
  - 実行結果、最終状態、履歴

## 中核クラス

- `Simulator`
  - 共通の実行ループ
  - `snapshot()`
  - `step()`
  - `run()`
- `CollisionAvoidanceAlgorithm`
  - `step(snapshot)`
- `AlgorithmRegistry`
  - `create()`
  - `list_algorithms()`

## 設計意図

- upstream よりも責務を絞った小さい API から始める
- `Simulator` は共通の実行ループに専念し、アルゴリズム固有判断は外へ出す
- アルゴリズムは `orca` `proxemic` `cnav` のように名前で差し替えられるようにする
- 同一シナリオを複数アルゴリズムへ流し、比較しやすくする

## 現時点の API 例

```python
from cmis_ca.algorithms.registry import create_algorithm
from cmis_ca.core.agent import AgentConfig, AgentProfile
from cmis_ca.core.geometry import Vector2
from cmis_ca.core.simulation import Simulator
from cmis_ca.core.world import Scenario

scenario = Scenario(
    agents=(
        AgentConfig(
            profile=AgentProfile(radius=0.4, max_speed=1.0),
            initial_position=Vector2(0.0, 0.0),
            preferred_velocity=Vector2(1.0, 0.0),
        ),
    ),
    time_step=0.5,
    steps=1,
)
algorithm = create_algorithm("orca")
simulator = Simulator(scenario=scenario, algorithm=algorithm)
result = simulator.run()
```

## 将来候補

- 障害物 API の拡張
- シナリオローダ
- ログ出力
- upstream 比較ユーティリティ
- バッチ比較実験 API
- 可視化 API

## 命名方針

- ソースコード上の識別子は Python の慣例に沿って英語ベースとする
- ドキュメント、コメント、説明文は日本語を基本とする
- 実装済みの振る舞いは `docs/specifications/` に記録し、設計意図と現状を混同しない

現時点の詳細仕様は [../specifications/python-skeleton-detailed-design.md](../specifications/python-skeleton-detailed-design.md) を参照する。
