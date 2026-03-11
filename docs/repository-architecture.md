# リポジトリ構造案

## 目的

このリポジトリは ORCA を単独で実装する場ではなく、ORCA を基準実装として、proxemic や CNav などの関連アルゴリズムを同一基盤上で比較・拡張できる形にする。  
そのため、リポジトリ構造は `アルゴリズムごとの分離` よりも、`共通コアと差分の分離` を優先する。

## 設計原則

1. 公開パッケージ名はアルゴリズム名に依存しない中立名とする
2. 共通コアは `core/` に集約し、ORCA 固有ロジックは `algorithms/orca/` に閉じ込める
3. proxemic や CNav も同じインターフェースで差し替えられるようにする
4. CLI では `--algorithm` だけで実行対象を切り替えられるようにする
5. シナリオ、設定、評価、可視化はアルゴリズム横断で共通化する

## 推奨パッケージ名

- リポジトリ名: `cmis-collision-avoidance`
- Python パッケージ名: `cmis_ca`
- CLI 名: `cmis-ca`

`cmis_orca` のような名前にすると、proxemic や CNav を追加した時に構造が不自然になるため避ける。

## 推奨ディレクトリ構造

```text
.
├── docs/
│   ├── design-overview.md
│   ├── repository-architecture.md
│   ├── api-draft.md
│   ├── source-file-policy.md
│   ├── issues/
│   └── algorithms/
│       ├── orca.md
│       ├── proxemic.md
│       └── cnav.md
├── external/
│   └── RVO2/                         # upstream 参照コード
├── scenarios/                       # YAML/JSON などのシナリオ定義
├── configs/
│   ├── defaults.yaml
│   └── algorithms/
│       ├── orca.yaml
│       ├── proxemic.yaml
│       └── cnav.yaml
├── src/
│   └── cmis_ca/
│       ├── __init__.py
│       ├── cli/
│       │   ├── main.py
│       │   ├── run.py
│       │   ├── benchmark.py
│       │   └── compare.py
│       ├── core/
│       │   ├── agent.py
│       │   ├── geometry.py
│       │   ├── state.py
│       │   ├── scenario.py
│       │   ├── world.py
│       │   ├── neighbor_search.py
│       │   ├── constraints.py
│       │   ├── solver.py
│       │   └── simulation.py
│       ├── algorithms/
│       │   ├── base.py
│       │   ├── registry.py
│       │   ├── orca/
│       │   │   ├── algorithm.py
│       │   │   ├── parameters.py
│       │   │   └── constraints.py
│       │   ├── proxemic/
│       │   │   ├── algorithm.py
│       │   │   ├── parameters.py
│       │   │   └── costs.py
│       │   └── cnav/
│       │       ├── algorithm.py
│       │       ├── parameters.py
│       │       └── planner.py
│       ├── io/
│       │   ├── scenario_loader.py
│       │   └── result_writer.py
│       ├── metrics/
│       │   ├── safety.py
│       │   ├── smoothness.py
│       │   └── efficiency.py
│       └── visualization/
│           ├── plot.py
│           └── animation.py
├── tests/
│   ├── core/
│   ├── algorithms/
│   │   ├── test_orca.py
│   │   ├── test_proxemic.py
│   │   └── test_cnav.py
│   ├── integration/
│   └── regression/
├── scripts/
└── pyproject.toml
```

## 共通コアとアルゴリズム差分

### `core/` に置くもの

- 2 次元ベクトルや幾何演算
- エージェント状態
- シナリオとワールド状態
- 近傍探索
- 制約表現
- 共有可能な線形計画ソルバや補助計算
- ステップ更新ループ

### `algorithms/` に置くもの

- ORCA 特有の制約生成
- proxemic 特有の重み付けやコスト
- CNav 特有の経路計画や意思決定
- アルゴリズム固有のパラメータ定義

判断基準は、複数アルゴリズムで再利用できるかどうかに置く。  
再利用できるなら `core/`、できないなら `algorithms/<name>/` に置く。

## アルゴリズム切替モデル

CLI と Python API の両方で、アルゴリズム名による切替を可能にする。

### CLI 例

```bash
cmis-ca run --algorithm orca --scenario scenarios/circle.yaml
cmis-ca run --algorithm proxemic --scenario scenarios/circle.yaml
cmis-ca run --algorithm cnav --scenario scenarios/circle.yaml
```

### Python API 例

```python
from cmis_ca.algorithms.registry import create_algorithm
from cmis_ca.core.simulation import Simulator

algorithm = create_algorithm("orca")
simulator = Simulator(algorithm=algorithm)
result = simulator.run(scenario)
```

## アルゴリズムインターフェース案

各アルゴリズムは共通の抽象インターフェースを実装する。

最低限の責務は以下。

- 初期化時にパラメータを受け取る
- 1 ステップ分の意思決定を返す
- 必要なら前処理や後処理を差し込める

たとえば `base.py` に以下のような Protocol を置く。

```python
class CollisionAvoidanceAlgorithm(Protocol):
    name: str

    def step(self, snapshot: WorldSnapshot) -> list[AgentCommand]:
        ...
```

この形にしておくと、`Simulator` 本体はアルゴリズム固有名を知らずに済む。

## シナリオと設定

1 コマンド切替を成立させるには、アルゴリズムとシナリオを分離して管理する必要がある。

- `scenarios/`: エージェント配置、目標、障害物など
- `configs/algorithms/`: アルゴリズムごとのデフォルトパラメータ
- `configs/defaults.yaml`: 共通設定

これにより、同じシナリオを `orca` `proxemic` `cnav` で共通に流せる。

## テスト方針

- `tests/core/`: 幾何演算や近傍探索などの共通部
- `tests/algorithms/`: アルゴリズム固有の単体テスト
- `tests/integration/`: CLI と実行導線
- `tests/regression/`: 既知シナリオに対する回帰チェック

比較実験を意識するなら、`tests/regression/` に同一シナリオでの過去結果との差分確認を置ける構造にしておくべき。

## ドキュメント方針

- `docs/design-overview.md`: 全体方針
- `docs/repository-architecture.md`: ディレクトリ構造と責務分担
- `docs/api-draft.md`: Python API / CLI の公開面
- `docs/algorithms/*.md`: 各アルゴリズムの理論、差分、実装メモ
- `docs/issues/*.md`: 作業単位の管理

## 現在の issue との関係

既存の `docs/issues/0001-python-bootstrap.md` は Python 初期移行の issue だが、先にこの多アルゴリズム構造を確定してから着手する方がよい。  
そのため、issue は一時保留にし、この構造案を基準に再定義してから再開する。
