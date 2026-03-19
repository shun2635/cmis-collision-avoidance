# CNav validation scenario 仕様

## 1. 目的

この文書は、issue `0029` で追加した CNav validation scenario の役割と固定条件を記録する。  
対象は trace parity と scenario parity の初手比較で使う最小ケースであり、smoke 実行用 scenario とは役割を分ける。

## 2. 基本方針

- validation scenario は `scenarios/` に YAML で置く
- 初手は fixed-step の trace 比較を優先する
- 1 scenario 1 観測目的を基本にする
- すべての agent に `goal_position` を与える

## 3. scenario 一覧

### 3.1 `scenarios/cnav_queue_validation.yaml`

- 目的: 狭い通路の隙間で、逆向き 2 agent の譲り行動を確認する
- 構成: 2 agent, 逆方向 goal, `compromise.cpp` 系の狭い通路障害物を縮小配置
- 障害物条件: 中央の隙間幅は agent 直径の `1.5` 倍
- 停止条件: `steps = 1000`
  - 実際の短い trace 比較は `--steps` override で切り詰める
- 主な観測指標:
  - ranked neighbors
  - chosen action
  - intended velocity
  - 狭隘部進入時の減速または回避

### 3.2 `scenarios/cnav_head_on_validation.yaml`

- 目的: 2 agent の対向で action 選択と ORCA 投影後速度を見る
- 構成: 2 agent, 反対 goal, `compromise.cpp` 系の狭い通路障害物を縮小配置
- 停止条件: `steps = 1000`
  - 実際の短い trace 比較は `--steps` override で切り詰める
- 主な観測指標:
  - chosen action
  - ORCA 後 velocity
  - 最小距離

### 3.3 `scenarios/cnav_crossing_validation.yaml`

- 目的: 4-way crossing で ranking と politeness の効き方を確認する
- 構成: 4 agent, 中央交差, `intersectionLoop.cpp` 系の中央交差障害物を縮小配置
- 停止条件: `steps = 1000`
  - 実際の短い trace 比較は `--steps` override で切り詰める
- 主な観測指標:
  - ranked neighbors
  - action ごとの reward
  - 中央付近での停滞有無

### 3.4 `scenarios/cnav_obstacle_validation.yaml`

- 目的: obstacle 制約と CNav coordination の相互作用を見る
- 構成: 2 agent, 中央障害物 1 つ
- 停止条件: `steps = 1000`
  - 実際の短い trace 比較は `--steps` override で切り詰める
- 主な観測指標:
  - intended velocity
  - ORCA obstacle constraint 後 velocity
  - obstacle 手前での振る舞い

## 4. smoke scenario との分離

- smoke 用: `scenarios/cnav_queue.yaml`
  - CLI 導線と最小実行確認
- validation 用:
  - `scenarios/cnav_queue_validation.yaml`
  - `scenarios/cnav_head_on_validation.yaml`
  - `scenarios/cnav_crossing_validation.yaml`
  - `scenarios/cnav_obstacle_validation.yaml`

validation scenario は、「動くこと」ではなく「何を比較するか」を固定するために使う。

## 5. 後続 issue への引き継ぎ

- issue `0030`
  - 各 scenario で trace dump を取る
- issue `0031`
  - 各 scenario の結果から parity gap を分類する

大規模 crowd や stochastic 更新比較が必要になった場合は、別 issue で拡張する。
