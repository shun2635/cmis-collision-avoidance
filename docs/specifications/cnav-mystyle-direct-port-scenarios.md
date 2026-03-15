# CNav MyStyle direct-port scenario 仕様

## 1. 目的

この文書は、`external/CNav_MyStyle` の setup を現行 scenario schema でできるだけ忠実に再現した direct-port scenario を記録する。  
対象は `2026-03-14` 時点の

- `external/CNav_MyStyle/simulations/forPaper.cpp`
- `external/CNav_MyStyle/simulations/crowdForPaper.cpp`

である。

## 2. 方針

- 既存の 2-4 agent validation scenario は維持する
- direct-port scenario は `初期配置`, `障害物形状`, `time_step`, `第1目標`, `agent profile` を legacy driver に揃える
- 現行 YAML schema で表現できない `changeDest()` と `temporary goal` は、未再現として明示する

## 3. scenario 一覧

### 3.1 `scenarios/cnav_forpaper_direct_port.yaml`

- 参照元: `external/CNav_MyStyle/simulations/forPaper.cpp`
- 構成: 12 agent, 9x9 cell field 由来の 36 obstacle cell
- 固定条件:
  - `time_step: 1.0`
  - `stop_when_all_agents_reach_goals: true`
  - agent profile は `neighbor_dist=100`, `max_neighbors=9`, `time_horizon=10`, `radius=10`, `max_speed=1.5`
- fidelity:
  - `searchCorrespondPosition()` によるセル中心座標を使用
  - 各 agent の初期位置オフセット `i * radius * 2.2` を反映
  - obstacle は `addObstaclesToSim()` と同じ 4 頂点順で定義
- limitation:
  - `dest1/dest2` の往復切替は未再現
  - `calculateTemporaryGoals()` による A* cell guidance は未再現

### 3.2 `scenarios/cnav_crowd_forpaper_direct_port.yaml`

- 参照元: `external/CNav_MyStyle/simulations/crowdForPaper.cpp`
- 構成: 50 agent, corridor wall 2 本
- 固定条件:
  - `time_step: 1.0`
  - `stop_when_all_agents_reach_goals: true`
  - agent profile は `neighbor_dist=100`, `max_neighbors=10`, `time_horizon=10`, `radius=5`, `max_speed=1.5`
- fidelity:
  - 5x5 の上下配置をそのまま移植
  - 長尺 wall polygon を元コードの頂点順で定義
- limitation:
  - 17 action set や slow-action speed 差分は scenario ではなく algorithm 側の gap として残る

## 4. 既存 validation scenario との役割分担

- `scenarios/cnav_queue_validation.yaml` など:
  - 分岐観測用の最小ケース
- `scenarios/cnav_forpaper_direct_port.yaml`
- `scenarios/cnav_crowd_forpaper_direct_port.yaml`
  - legacy setup fidelity を上げる比較ケース

両者は競合ではなく、比較粒度が異なる。

## 5. 残差分

direct-port scenario を追加しても、次の差分は残る。

- temporary goal / cell guidance
- `changeDest()` による往復目標切替
- 17 action set
- legacy reward / politeness pipeline

したがって direct parity を主張するときは、「scenario setup は揃えたが guidance / action model は未一致」と明記する。

## 6. 実行例

```bash
poetry run cmis-ca visualize --algorithm cnav --scenario scenarios/cnav_forpaper_direct_port.yaml
poetry run cmis-ca visualize --algorithm cnav --scenario scenarios/cnav_crowd_forpaper_direct_port.yaml
poetry run python scripts/dump_cnav_trace.py scenarios/cnav_forpaper_direct_port.yaml --steps 2 --profile legacy-forpaper-comparison
```
