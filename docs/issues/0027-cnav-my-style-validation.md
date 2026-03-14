# Issue 0027: `external/CNav_MyStyle` を基準に CNav を再検証する

- ステータス: todo
- 優先度: high
- 関連文書:
  - [docs/algorithms/cnav.md](../algorithms/cnav.md)
  - [docs/specifications/cnav-initial-implementation.md](../specifications/cnav-initial-implementation.md)
  - [README.md](../../README.md)

## 背景

現行の Python 実装は、`2020` の C-Nav 論文に寄せた最小導入として作っている。  
一方で、今回追加した `external/CNav_MyStyle` は、過去に動かしていた C++ 実装であり、実際の研究用挙動はむしろこちらに近い可能性がある。

ただし `external/CNav_MyStyle` を見ると、論文どおりと断言できない追加ヒューリスティクスや driver ごとの差分も入っている。

- `src/simulationDefs.h` では `coordFactor=0.1`、`updateProbability=0.25`、`allNeigh=1`
- `src/RVOSimulator.cpp` の `SimulateVelocity(...)` には `sameWeight`、`counterDirection`、`slowMove`、`maxNumSimulateNeighbors`、`maxNumThinkNeighbors` などの追加分岐がある
- `examples/circle.cpp` は確率更新を使うが、`simulations/forPaper.cpp` は毎 step 再評価に近い流れになっている

このため、現時点では「Python 実装が期待どおりか」を直接判断できない。  
まずは `external/CNav_MyStyle` のどの振る舞いを正本にするかを固定し、その上で trace ベースに再検証する必要がある。

## 目的

`external/CNav_MyStyle` を比較基準として、現行 Python 実装の CNav が

- 論文寄りの再現をしているのか
- `CNav_MyStyle` の実挙動を再現しているのか
- どちらでもない差分を持っているのか

を切り分けられる検証手順を定義する。

## スコープ

- `external/CNav_MyStyle` 内の正本 entry point とパラメータの特定
- action 更新周期、neighbor 選別、reward 計算、communication timing の比較観点固定
- 小規模 scenario での trace 比較方針の策定
- Python 側で追加すべきテスト / 計測 hook の整理

## 非スコープ

- この issue 単体での完全移植
- 大規模 benchmark の実施
- GUI の見た目再現
- どの差分も即座に Python 側へ取り込むこと

## 完了条件

- `external/CNav_MyStyle` の生成物が `.gitignore` で無視される
- 比較対象にする C++ driver と固定パラメータが文書化されている
- unit / trace / scenario の 3 段階の検証手順が定義されている
- 現行 Python 実装との主要な差分仮説が列挙されている
- 次の実装作業が `fix`, `intentional difference`, `legacy-only heuristic` のどれに当たるか判定できる

## 想定成果物

- `.gitignore`
- 本 issue 文書
- 今後追加する比較用 scenario / trace dump / parity tests

## アルゴリズム検証方針

### 1. 基準系の固定

最初に、`external/CNav_MyStyle` のどの driver を「比較元」として使うかを固定する。  
理由は、同ディレクトリ内でも更新規則やフラグが一致していないためである。

第一候補:

- `simulations/forPaper.cpp`
- `simulations/crowdForPaper.cpp`
- `examples/circle.cpp`

進め方:

- まず `forPaper` 系を paper-facing baseline として読む
- `circle.cpp` や `crowd.cpp` は差分確認用の secondary baseline とする
- baseline ごとに `coordFactor`、`allNeigh`、`lengthSimulateTimeSteps`、action 更新条件、neighbor 上限を固定表にする

### 2. 比較単位の分解

いきなり full simulation の到達時間だけを比べない。  
以下の順で、内側から外側へ比較する。

1. action update cadence
2. constrained neighbor selection
3. 1 action あたりの reward 計算
4. chosen action から intended velocity への変換
5. ORCA 通過後の最終速度
6. 複数 step の行動列と到達結果

この順で比較すると、「差分が CNav 本体なのか ORCA 接続なのか」を切り分けやすい。

### 3. unit レベルの確認項目

Python 側で最初に固定する確認項目:

- action 集合の順序が legacy 実装と一致するか
- 更新周期が `0.2s 固定` なのか `updateProbability=0.25` 相当なのか
- neighbor の対象が `goal に近い側のみ` なのか `allNeigh` なのか
- politeness が `||v_intent - v_new||` 系で評価されているか
- `maxNumSimulateNeighbors` と `maxNumThinkNeighbors` の上限が効いているか
- `sameWeight` や反対方向判定などのヒューリスティクスが実挙動へ効いているか

### 4. trace レベルの比較

小規模 scenario を 2-4 体で用意し、各 step で次を比較できる状態にする。

- ranked neighbors
- action ごとの `goal_progress`
- action ごとの `politeness`
- action ごとの total reward
- chosen action index
- intended velocity
- ORCA 後の velocity

比較は数値 1 個だけではなく、`step x agent x action` の表で残す。  
そうしないと、終端結果だけ偶然一致しているケースを見落とす。

### 5. scenario レベルの比較

最初の比較対象は、大規模 crowd ではなく差分を説明しやすい最小ケースに絞る。

- 同一 goal への 2 agent queue
- 交差 2x2
- head-on 2 agent
- 単一障害物つき 2-3 agent

見る指標:

- chosen action の時系列
- goal 到達時間
- 停滞の有無
- 最小距離
- collision 発生数

### 6. 判定ルール

比較の結果出た差分は、次の 3 種に分けて扱う。

1. paper からの逸脱で、legacy 側固有の実験ヒューリスティクス
2. Python 実装の欠落または誤実装
3. 意図的差分として docs に残すべきもの

以後の修正は、この分類を付けてから着手する。  
`legacy と違う` だけでは、その差分を取り込む理由にならない。

## 初手で確認する差分仮説

- 現行 Python の既定 `coordination_factor=0.8` は、legacy 既定 `coordFactor=0.1` と大きく異なる
- 現行 Python は `action_update_interval=0.2` の固定周期だが、legacy には確率更新の driver がある
- 現行 Python は `goal に近い neighbor のみ` を標準化しているが、legacy には `allNeigh` 分岐が残っている
- 現行 Python の reward は論文式で正規化しているが、legacy は追加ヒューリスティクスと neighbor 上限制御を混ぜている
- 現行 Python の tests は unit 寄りで、legacy との trace parity をまだ持っていない

## 作業メモ

- `external/CNav_MyStyle` は source reference と生成物が混在しているため、まず ignore を整理してから読む
- 先に `paper baseline` と `my-style baseline` を分けて管理する
- 比較結果は最終的に `docs/specifications/` へ固定し、Python 実装の仕様と分離する
