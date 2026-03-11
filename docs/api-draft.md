# API 草案

## 目的

upstream の概念を踏まえつつ、研究室内で扱いやすい Python ベースの最小 API を定義する。  
初期段階では、教材・比較実験・将来のアルゴリズム追加に耐える単純な構成を優先する。

## 想定する公開 API

### Python パッケージ

`cmis_ca`

### CLI

`cmis-ca`

### CLI の基本形

```bash
cmis-ca run --algorithm orca --scenario scenarios/circle.yaml
cmis-ca run --algorithm proxemic --scenario scenarios/circle.yaml
cmis-ca run --algorithm cnav --scenario scenarios/circle.yaml
```

### 基本データ型

- `Vector2`
  - 2 次元ベクトル
- `Agent`
  - エージェント設定
- `AgentConfig`
  - 半径、最大速度、初期位置、初期速度などの設定
- `AgentState`
  - 現在位置、現在速度、希望速度
- `Scenario`
  - 初期配置、目標、障害物、タイムステップなど
- `SimulationResult`
  - 軌跡、メトリクス、ログなど

### 中核クラス

- `Simulator`
  - `step`
  - `run`
- `CollisionAvoidanceAlgorithm`
  - `step`
- `AlgorithmRegistry`
  - `create`
  - `list_algorithms`

## 設計意図

- upstream よりも責務を絞った小さい API から始める
- `Simulator` は共通の実行ループに専念し、アルゴリズム固有判断は外へ出す
- アルゴリズムは `orca` `proxemic` `cnav` のように名前で差し替えられるようにする
- 同一シナリオを複数アルゴリズムへ流し、比較しやすくする

## 想定 API 例

```python
from cmis_ca.algorithms.registry import create_algorithm
from cmis_ca.core.scenario import Scenario
from cmis_ca.core.simulation import Simulator

scenario = Scenario(...)
algorithm = create_algorithm("orca")
simulator = Simulator(algorithm=algorithm)
result = simulator.run(scenario)
```

## 将来候補

- 障害物 API
- シナリオローダ
- ログ出力
- upstream 比較ユーティリティ
- バッチ比較実験 API
- 可視化 API

## 命名方針

- ソースコード上の識別子は Python の慣例に沿って英語ベースとする
- ドキュメント、コメント、説明文は日本語を基本とする

この方針により、研究室内での可読性と、将来的な外部連携のしやすさを両立します。
