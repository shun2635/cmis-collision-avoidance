# CNav MyStyle baseline 監査

## 1. 目的

この文書は、`external/CNav_MyStyle` を CNav legacy 実装の比較元として使う際に、どの driver を baseline とみなすかを固定する。  
対象は `2026-03-14` 時点で repo に置かれている `external/CNav_MyStyle/` の内容である。

## 2. 結論

比較基準は次の 3 層で固定する。

### 2.1 primary baseline

- `external/CNav_MyStyle/simulations/forPaper.cpp`

採用理由:

- file 名と構成が paper-facing driver として最も明示的
- `main()` の既定実行が 1 本に絞られており、比較条件を固定しやすい
- action 更新が `timeStep % 1 == 0` の deterministic cadence で進む
- `allNeigh = false` を採用しており、論文整理と整合しやすい
- `maxNumSimulateNeighbors`、`maxNumThinkNeighbors`、`sameWeight` を含む current MyStyle 系の reward pipeline を持つ

### 2.2 secondary baseline

- `external/CNav_MyStyle/simulations/crowdForPaper.cpp`

用途:

- `forPaper.cpp` と同じ reward / neighbor 制御系を、単純な corridor crowd で確認するための補助 baseline
- 障害物つき symmetric crowd に対する validation scenario 設計の参照元

### 2.3 legacy experimental baseline

- `external/CNav_MyStyle/examples/circle.cpp`

用途:

- 確率更新 `updateProbability = 0.25` を含む旧系統の比較用
- `forPaper` 系と異なる更新規則の存在確認用

非採用理由:

- action 更新が deterministic ではなく確率的
- `SimulateVelocity(...)` 呼び出しが旧シグネチャ寄りで、`maxNumSimulateNeighbors` や `sameWeight` を渡していない
- scenario / action 構成が現行 MyStyle 本流より単純で、primary baseline にすると後続 issue がぶれる

## 3. driver ごとの差分表

| 種別 | `forPaper.cpp` | `crowdForPaper.cpp` | `examples/circle.cpp` |
| --- | --- | --- | --- |
| 位置付け | primary baseline | secondary baseline | legacy experimental baseline |
| `main()` 既定呼び出し | `mainLoop(1,0.9f,2,3,3,1,10)` | `mainLoop(1,0.9f,10,3,5,1,10)` | `coordFactor=0.1f`, `allNeigh=0`, `finalIteration=10` |
| action 更新 cadence | 毎 step 判定、`timeStep > 1` で毎回再評価 | 毎 step 判定、`timeStep > 1` で毎回再評価 | `updateProbability = 0.25` の確率更新 |
| `coordFactor` の既定 | `0.9` | `0.9` | `0.1` |
| `allNeigh` | `false` | `false` | `0` |
| `lengthSimulateTimeSteps` | 呼び出し既定は `3` | 呼び出し既定は `3` | file 既定 `2` |
| simulate neighbors 上限 | `3` | `5` | なし |
| think neighbors 上限 | `1` | `1` | `contadourX=1` のみ |
| `sameWeight` | `10` | `10` | なし |
| action 数 | `17` | `17` | `8` または `9` |
| action speed | `1.5 x 8`, `0.75 x 8`, `0.0 x 1` | `1.5 x 8`, `0.1 x 8`, `0.0 x 1` | `1.5 x 8`、追従ありなら `9` |
| scenario | 9x9 cell field の 12 agent + obstacle | corridor crowd の 50 agent + obstacle | circle の 5 agent |
| temporary goal | あり | あり | なし |
| trace 出力 | position のみ既定有効 | action/prefV/position path あり | result/position path あり |

## 4. `simulationDefs.h` の扱い

`external/CNav_MyStyle/src/simulationDefs.h` には次の既定値がある。

- `updateProbability = 0.25`
- `coordFactor = 0.1`
- `lengthSimulateTimeSteps = 2`
- `allNeigh = 1`

ただし、これは `external/CNav_MyStyle` 全体の正本パラメータではない。  
実際には driver 側で次が上書きされている。

- `forPaper.cpp`: `coordFactor = 0.9`, `allNeigh = false`, deterministic 毎 step 更新
- `crowdForPaper.cpp`: `coordFactor = 0.9`, `allNeigh = false`, deterministic 毎 step 更新
- `circle.cpp`: `coordFactor = 0.1`, `allNeigh = 0`, `updateProbability = 0.25`

したがって、後続 issue で baseline を参照するときは `simulationDefs.h` 単体ではなく、必ず driver の `main()` と `mainLoop(...)` 呼び出しを優先する。

## 5. primary baseline の固定条件

`forPaper.cpp` を比較元に使うときの固定条件:

- algorithm: `2` (`C-Nav`)
- `coordFactor = 0.9`
- `allNeigh = false`
- `contadourX = 1`
- `lengthSimulateTimeSteps = 3`
- `maxNumSimulateNeighbors = 3`
- `maxNumThinkNeighbors = 1`
- `sameWeight = 10`
- action set: 17 actions
- update cadence: `timeStep > 1` 以降の毎 step 再評価

この baseline は、論文そのものの正本ではなく `MyStyle current baseline` として扱う。

## 6. secondary baseline の固定条件

`crowdForPaper.cpp` を比較元に使うときの固定条件:

- algorithm: `2`
- `coordFactor = 0.9`
- `allNeigh = false`
- `contadourX = 1`
- `lengthSimulateTimeSteps = 3`
- `maxNumSimulateNeighbors = 5`
- `maxNumThinkNeighbors = 1`
- `sameWeight = 10`
- action set: 17 actions
- update cadence: `timeStep > 1` 以降の毎 step 再評価

`forPaper.cpp` との差分は、主に scenario 構造と slow action speed の設定にある。

## 7. validation scenario への引き継ぎ

issue `0029` では、次の対応で scenario を作る。

- `forPaper.cpp` 由来:
  - temporary goal と obstacle interaction を見る validation case
- `crowdForPaper.cpp` 由来:
  - corridor crossing / bidirectional crowd を簡略化した validation case
- `circle.cpp` 由来:
  - 確率更新比較をやりたくなった場合のみ別系統で扱う

初手の validation scenario は `forPaper.cpp` と `crowdForPaper.cpp` を正とし、`circle.cpp` の確率更新は後段オプションとする。

## 8. parity 判定上の注意

- `forPaper` 系は Python 現行実装の 8 action 固定と一致しない
- `forPaper` 系は `coordFactor = 0.9` を既定にしており、Python 既定 `0.8` とずれる
- `forPaper` 系は deterministic 毎 step 更新であり、Python の `action_update_interval = 0.2` と直接一致しない
- `circle.cpp` は `updateProbability = 0.25` を使うため、`0.2s` との比較は別 issue で切り分ける

したがって後続 parity では、まず

1. `forPaper` 系に合わせる比較
2. 論文寄り Python 既定との差分分類

の順で進める。
