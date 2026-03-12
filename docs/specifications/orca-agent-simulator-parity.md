# ORCA agent / simulator parity 監査

## 1. 目的

この文書は、upstream `RVO2` の `Agent` / `RVOSimulator` と現行 Python 実装の差分を監査し、どこまで追従し、どこを未対応として残しているかを固定する。

## 2. 参照元

- `external/RVO2/src/RVOSimulator.h`
- `external/RVO2/src/RVOSimulator.cc`
- `external/RVO2/src/Agent.h`
- `external/RVO2/src/Agent.cc`

## 3. 差分表

| upstream 概念 | upstream の位置 | Python 側の現行位置 | 現在の扱い |
| --- | --- | --- | --- |
| `timeStep_` | `RVOSimulator` | `Scenario.time_step` | 実装済み |
| `globalTime_` | `RVOSimulator` | `Simulator.global_time`, `WorldSnapshot.global_time` | 実装済み |
| `defaultAgent_` | `RVOSimulator` | `AgentProfile` の既定値 | 明示オブジェクトではなく dataclass default で表現 |
| `neighborDist_` | `Agent` | `AgentProfile.neighbor_dist` | 実装済み |
| `maxNeighbors_` | `Agent` | `AgentProfile.max_neighbors` | 実装済み |
| `timeHorizon_` | `Agent` | `AgentProfile.time_horizon` | 実装済み |
| `timeHorizonObst_` | `Agent` | `AgentProfile.time_horizon_obst` | 実装済み |
| `radius_` | `Agent` | `AgentProfile.radius` | 実装済み |
| `maxSpeed_` | `Agent` | `AgentProfile.max_speed` | 実装済み |
| `velocity_` 初期値 | `Agent` | `AgentConfig.initial_velocity` | 実装済み |
| `prefVelocity_` | `Agent` | `AgentState.preferred_velocity` | 実装済み |
| `setAgentDefaults(...)` | `RVOSimulator` | scenario 内 `AgentProfile` の共通記述 | API 形は異なるが意味は保持 |
| `setAgentPrefVelocity(...)` | `RVOSimulator` | `Simulator.set_preferred_velocity()` | 実装済み |
| goal に基づく `prefVelocity` 更新 | example code 側 | `Simulator.refresh_preferred_velocities_from_goals()` | 実装済み |

## 4. 現在の設計判断

- upstream では ORCA 関連 parameter は agent instance にぶら下がるため、Python 側でも `AgentProfile` に保持する
- `ORCAParameters` は agent profile を置き換えるものではなく、algorithm 側の optional override として扱う
- `global_time` は simulator state と snapshot の両方へ持たせ、upstream の `getGlobalTime()` に相当する読み出し口を確保する

## 5. 未対応として残す差分

- upstream の obstacle graph / convexity は未対応
- upstream の kd-tree 実装は未対応で、近傍探索は `NaiveNeighborSearch`
- `RVOSimulator` の setter 群をそのまま Python API に写してはいない
- default agent object の生成・複製モデルは dataclass default へ簡略化している

## 6. 現時点の結論

agent / simulator semantics については、ORCA 再現に直結する主要 parameter と clock は Python 側へ反映済みである。  
次段階の主要 gap は obstacle topology と obstacle constraint の upstream 準拠化である。
