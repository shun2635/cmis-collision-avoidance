# CMIS Collision Avoidance

研究室向けの collision avoidance 実験リポジトリです。  
現在は Python 実装を正本として、`orca` と `cnav` を CLI から実行できます。

- 正規実装: `src/cmis_ca/`
- 利用可能アルゴリズム: `orca`, `cnav`
- scenario 形式: YAML / JSON
- 可視化: `cmis-ca visualize`

`external/RVO2` は ORCA の参照元、`external/CNav_MyStyle` は過去の CNav 実装の比較元です。  
third-party code の扱いは [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md) を参照してください。

## 必要環境

- Python `>=3.9,<4.0`
- Poetry

`visualize` を使う場合は `numpy` / `pyqtgraph` / `PySide6` が入り、Python `>=3.15` では viewer は未対応です。  
`--save-animation` で `.mp4` / `.gif` を出す場合は `ffmpeg` も必要です。

## セットアップ

```bash
poetry install
```

## まず動かす

### 1. built-in demo を可視化する

```bash
poetry run cmis-ca visualize --algorithm orca
poetry run cmis-ca visualize --algorithm cnav
```

`--scenario` を省略すると built-in の `circle-demo` を使います。  
既定 step 数は `1000` です。

### 2. 最小 scenario を可視化する

```bash
poetry run cmis-ca visualize --algorithm orca --scenario scenarios/head_on.yaml
poetry run cmis-ca visualize --algorithm cnav --scenario scenarios/cnav_queue.yaml
poetry run cmis-ca visualize --algorithm cnav --scenario scenarios/cnav_queue_validation.yaml --save-animation artifacts/cnav_queue_validation.mp4
```

### 3. 数値だけ確認したい場合

```bash
poetry run cmis-ca run --algorithm orca --scenario scenarios/head_on.yaml
poetry run cmis-ca run --algorithm cnav --scenario scenarios/cnav_queue.yaml
```

### 4. テストする

```bash
poetry run pytest
```

## CLI の使い方

```bash
poetry run cmis-ca run --algorithm <orca|cnav> [--scenario PATH] [--steps N]
poetry run cmis-ca visualize --algorithm <orca|cnav> [--scenario PATH] [--steps N] [--fps FPS] [--save-animation PATH]
poetry run cmis-ca visualize --algorithm cnav --scenario PATH --cnav-profile legacy-forpaper-decision
poetry run cmis-ca visualize --algorithm cnav --scenario PATH --cnav-mystyle-driver forpaper
```

- `--scenario` を指定しない: built-in `circle-demo`
- `--scenario` を指定する: scenario ファイルの `steps` / 停止条件を使う
- `--steps` を指定する: scenario 設定より優先
- `--cnav-profile`: CNav の decision profile だけを切り替える
- `--cnav-mystyle-driver`: `external/CNav_MyStyle` の driver 固定値で `time_step` と agent profile までまとめて上書きする
- `--save-animation`: trace を `.mp4` または `.gif` に保存してから viewer を開く

## 同梱 scenario

### `scenarios/head_on.yaml`

2 agent の正面衝突回避を見る最小 ORCA ケースです。

```bash
poetry run cmis-ca visualize --algorithm orca --scenario scenarios/head_on.yaml
```

### `scenarios/obstacle_demo.yaml`

単一障害物に対する回避挙動を見る最小 ORCA ケースです。

```bash
poetry run cmis-ca visualize --algorithm orca --scenario scenarios/obstacle_demo.yaml
```

### `scenarios/cnav_queue.yaml`

同一 goal に向かう 2 agent の queue を使った CNav の最小 smoke ケースです。

```bash
poetry run cmis-ca visualize --algorithm cnav --scenario scenarios/cnav_queue.yaml
```

### `scenarios/cnav_queue_validation.yaml`

`compromise.cpp` 系の狭い通路で、逆向き 2 agent が `直径 * 1.5` の隙間を通るときの譲り行動を見る validation ケースです。

```bash
poetry run cmis-ca visualize --algorithm cnav --scenario scenarios/cnav_queue_validation.yaml
```

### `scenarios/cnav_head_on_validation.yaml`

`compromise.cpp` 系の狭い通路で、2 agent 対向時の action selection と ORCA 投影後速度を見る validation ケースです。

```bash
poetry run cmis-ca visualize --algorithm cnav --scenario scenarios/cnav_head_on_validation.yaml
```

### `scenarios/cnav_crossing_validation.yaml`

`intersectionLoop.cpp` 系の中央交差障害物つき 4-way crossing で ranking と politeness の効き方を見る validation ケースです。

```bash
poetry run cmis-ca visualize --algorithm cnav --scenario scenarios/cnav_crossing_validation.yaml
```

### `scenarios/cnav_obstacle_validation.yaml`

障害物制約と CNav coordination の相互作用を見る validation ケースです。

```bash
poetry run cmis-ca visualize --algorithm cnav --scenario scenarios/cnav_obstacle_validation.yaml
```

### `scenarios/cnav_forpaper_direct_port.yaml`

`external/CNav_MyStyle/simulations/forPaper.cpp` の setup を寄せた direct-port 比較ケースです。

```bash
poetry run cmis-ca visualize --algorithm cnav --scenario scenarios/cnav_forpaper_direct_port.yaml
poetry run cmis-ca visualize --algorithm cnav --scenario scenarios/cnav_forpaper_direct_port.yaml --cnav-mystyle-driver forpaper
```

### `scenarios/cnav_crowd_forpaper_direct_port.yaml`

`external/CNav_MyStyle/simulations/crowdForPaper.cpp` の corridor crowd setup を寄せた direct-port 比較ケースです。

```bash
poetry run cmis-ca visualize --algorithm cnav --scenario scenarios/cnav_crowd_forpaper_direct_port.yaml
poetry run cmis-ca visualize --algorithm cnav --scenario scenarios/cnav_crowd_forpaper_direct_port.yaml --cnav-mystyle-driver crowd-forpaper
```

### `scenarios/upstream_circle.yaml`

`external/RVO2/examples/Circle.cc` に基づく 250 agent の upstream 比較ケースです。

```bash
poetry run cmis-ca visualize --algorithm orca --scenario scenarios/upstream_circle.yaml
```

validation / direct-port scenario は既定 `steps` が長めです。短く確認したい場合は `--steps 10` のように上書きしてください。数値だけ見たい場合は `visualize` を `run` に置き換えてください。

## リポジトリ構成

```text
.
├── src/cmis_ca/      # Python 実装
├── scenarios/        # YAML / JSON scenario
├── tests/            # pytest
├── docs/             # 設計・仕様・issue
├── external/RVO2/    # ORCA 参照コード
└── external/CNav_MyStyle/  # CNav legacy 比較元
```

## 困ったときの参照先

- docs 一覧: [docs/README.md](docs/README.md)
- 現行実装の詳細: [docs/specifications/python-skeleton-detailed-design.md](docs/specifications/python-skeleton-detailed-design.md)
- CNav の概要: [docs/algorithms/cnav.md](docs/algorithms/cnav.md)
- CNav 初期実装仕様: [docs/specifications/cnav-initial-implementation.md](docs/specifications/cnav-initial-implementation.md)
- CNav validation scenario: [docs/specifications/cnav-validation-scenarios.md](docs/specifications/cnav-validation-scenarios.md)
- ORCA roadmap: [docs/architecture/orca-reproduction-roadmap.md](docs/architecture/orca-reproduction-roadmap.md)

## ライセンス

- この repo 全体: [LICENSE](LICENSE)
- third-party notices: [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md)
