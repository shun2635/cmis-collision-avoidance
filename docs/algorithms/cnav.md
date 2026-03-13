# CNav

## 位置付け

CNav は、このリポジトリで `ORCA` の次に追加したアルゴリズムである。  
ただし CNav は ORCA の代替ではなく、`離散 action から intended velocity を選ぶ上位の coordination layer` として扱う。  
実際の collision-free velocity は、CNav が選んだ `v_intent` を入力として ORCA が計算する。

したがって本リポジトリでは、CNav を `algorithms/cnav/` に実装しつつ、低レベルの衝突回避は既存の ORCA 実装を再利用する方針を取る。

## 正本にする出典

CNav まわりは関連論文が複数あるため、本リポジトリでは出典の役割を次のように固定する。

1. 原典
   - Julio Godoy, Ioannis Karamouzas, Stephen J. Guy, Maria Gini,
     "Implicit Coordination in Crowded Multi-Agent Navigation",
     AAAI 2016
   - URL: <https://motion.cs.umn.edu/r/CNAV/>
   - CNav 系列の最初の提案として扱う
2. 実装時の正本
   - Julio Godoy, Stephen J. Guy, Maria Gini, Ioannis Karamouzas,
     "C-Nav: Distributed Coordination in Crowded Multi-Agent Navigation",
     Robotics and Autonomous Systems 向け preprint, November 2020
   - URL: <https://www-users.cse.umn.edu/~gini/publications/papers/Godoy2020.pdf>
   - Algorithm 1-3 と Eq. (1)-(4) を含み、実装粒度が最も高いので、実装時の正本とする
3. 系譜上の参照
   - Julio Godoy, Ioannis Karamouzas, Stephen J. Guy, Maria Gini,
     "Adaptive Learning for Multi-Agent Navigation" (`ALAN` の初出),
     AAMAS 2015
   - URL: <https://motion.cs.umn.edu/r/ALAN/ALAN.pdf>
   - CNav が継承した action-selection の考え方を説明するための参照として扱う

以後、このリポジトリで「CNav のアルゴリズム」と言う場合は、`2016 の原典で提案された枠組みを 2020 の C-Nav 論文で詳細化したもの` を指す。  
実装判断で差分が出た場合は、`2020 C-Nav > 2016 AAAI > ALAN` の順で優先する。

## CNav の前提

2020 の C-Nav 論文では、各 agent は次を前提にしている。

- 2D 平面上を動く円盤 agent
- 半径 `r_i`
- 現在位置 `p_i`
- 現在速度 `v_i`
- 最大速度 `v_max`
- goal 方向を向く `v_goal`
- 離散 action 集合から選ばれる intended velocity `v_intent`
- 近傍の位置、速度、半径を sensing できる
- 近傍へ `agent ID` と `intended velocity` を one-way broadcast できる

ここで重要なのは、CNav 自体は collision avoidance を直接解かないことだ。  
各時刻でまず CNav が `v_intent` を選び、その後に ORCA が `v_new` を計算する。  
したがって CNav の本質は `離散 action の選択` と `近傍協調の評価` にある。

## 行動空間

C-Nav 論文では、goal 方向を基準にした固定 action 集合を使う。  
Figure 2 の既定は以下の 8 action である。

- `0°`
- `+β`
- `-β`
- `+90°`
- `-90°`
- `180°`
- `180° + β`
- `180° - β`

論文中の実装では `β = 45°`、speed は `1.5 m/s` で固定されている。  
このため、論文再現だけを優先する最初の実装では、action 集合をパラメータ化しつつ、既定値は論文どおりに持つのが自然である。

## 1 回の action 更新

CNav の action 更新は、毎 simulation step ではなく、数 step ごとに走る。  
論文では平均 `0.2 s` ごとに action を更新している。

更新時の処理は次のとおり。

1. `GetMostConstrainedNeighs(i)` で、agent `i` から見て最も constrained な neighbors を順位付けする
2. 各 action `a` について `SimMotion(i, a, C_rank)` を実行する
3. reward `R_a` を最大にする action を選び、それに対応する `v_intent` を採用する
4. 選ばれた `v_intent` を近傍へ broadcast する

action 更新が走らない step では、直前に選んだ `v_intent` をそのまま使い続ける。

## constrained neighbor の順位付け

論文の `GetMostConstrainedNeighs(i)` は、neighbor `j` の observed velocity と broadcast 済み intended velocity のずれ

- `||v_j^intent - v_j^new||`

を constraint の強さとして使う。  
ずれが大きいほど、その neighbor は「本来やりたい motion を十分に実行できていない」とみなされる。

ただし、すべての neighbor を対象にするわけではない。  
論文では circular dependency を避けるため、`自分より goal に近い neighbor` だけを順位付け対象にする。  
これにより、同じ goal に向かう agent 同士が同時に互いへ譲り続ける deadlock / livelock を減らす設計になっている。

## action 評価

各 action は短い horizon `T` だけ forward simulate される。  
論文では `T = 2` step を使っている。

各 simulated step では、agent 自身と近傍の状態を ORCA で更新しながら、次の 2 成分を蓄積する。

### 1. goal-progress 成分

agent 自身が goal 方向へどれだけ進めるかを評価する。

`R_ga = (1 / (T * v_max)) * Σ_t (v_i^new · normalize(g_i - p_i))`

この項は、`goal に速く近づく action` を好む。

### 2. constraint-reduction 成分

上位 `k` 個の constrained neighbors に対して、どれだけ motion constraint を減らせるかを評価する。

まず neighbor `j` に対する politeness を

`P_a,j = v_max - ||v_j^intent - v_j^new||`

と定義する。  
次に、それを上位 `k` neighbor と horizon 上で平均した

`R_ca = (1 / ((T - 1) * k * v_max)) * Σ_t Σ_j P_a,j`

を使う。

この項は、`neighbor の intended motion を妨げにくい action` を好む。

### 最終 reward

最終的な action 評価は、論文 Eq. (2) の

`R_a = (1 - γ) * R_ga + γ * R_ca`

で与えられる。  
`γ` は coordination factor であり、論文の分析では `γ = 0.8` が最も良い結果を示すケースが多い。

## 毎 simulation step の処理

action 更新の有無に関わらず、各 simulation step では最終的に

1. 現在保持している `v_intent` を ORCA へ渡す
2. ORCA が collision-free velocity `v_new` を返す
3. `position += v_new * dt` で state を更新する

という流れになる。  
したがって CNav の最小再現には、`ORCA を任意の intended velocity で呼び出せること` が必要である。

## ALAN との関係

CNav は ALAN と完全に同じアルゴリズムではない。  
ただし、action-selection の考え方には明確な系譜がある。

### ALAN から引き継ぐもの

- preferred velocity を離散 action 集合として扱う考え方
- `goal progress` と `politeness` の重み付き和で action を評価する考え方
- ORCA を低レベル collision avoidance として使う構成

### CNav で追加または変更されるもの

- neighbor の `intended velocity` を one-way communication で受け取る
- `自分より goal に近い neighbor` だけを見る順位付けを使う
- action を bandit learning ではなく short-horizon simulation で評価する
- politeness を `neighbor の intended velocity をどれだけ妨げるか` で定義する

### 最初の実装で持ち込まないもの

- ALAN の `wUCB` や context-aware exploration
- 履歴ベースの action value 推定
- ALAN 特有の exploration / exploitation 制御

つまり、本リポジトリで実装する CNav は `ALAN をそのまま移植する` のではなく、`ALAN から action-selection の骨格を受け継ぎつつ、2020 C-Nav 論文の通信付き simulation-based 選択を再現する` ものとする。

## この repo での実装境界

### `core/` に残すもの

- 幾何型
- scenario / world / snapshot
- neighbor search
- ORCA が使う line constraint と linear solver
- 共通 simulation loop

### `algorithms/cnav/` に置くもの

- CNav 固有パラメータ
- action set 定義
- constrained neighbor ranking
- simulated neighborhood update
- CNav reward 計算
- `v_intent` の保持と broadcast 状態

### ORCA 再利用の方針

CNav 実装では、`ORCAAlgorithm.step()` をそのまま呼ぶだけでは足りない。  
理由は、CNav では `snapshot.state.preferred_velocity` とは別に、agent ごとの `現在の intended velocity` を内部状態として保持する必要があるからである。

そのため検討時点では、次の 2 案があった。

1. ORCA の per-agent solve を helper 化し、`optimization_velocity` を明示的に渡せるようにする
2. CNav 側で ORCA の constraint build と solver 呼び出しを再構成する

### 採用方針

`1. ORCA の per-agent solve を helper 化する` を採用する。  
理由は次のとおり。

- ORCA の constraint build と solver 呼び出しを CNav 側へ重複実装せずに済む
- `ORCAAlgorithm.step()` の責務を壊さず、per-agent solve だけを再利用できる
- 将来 `proxemic` などが `任意の intended velocity を ORCA へ通す` 形を取る場合にも流用しやすい

したがって、最初に着手すべき実装は `ORCAAlgorithm.step()` 内部の per-agent solve を helper として切り出すことである。

## ドキュメント構造

CNav 導入時の docs は次の役割分担で管理する。

- 本文書 `docs/algorithms/cnav.md`
  - 理論、出典、ALAN との差分、実装境界
- `docs/issues/0022-cnav-initial-implementation.md`
  - 実装着手の issue と完了条件
- 実装後に追加する仕様書
  - `docs/specifications/` に初期 CNav 実装の固定仕様を書く
- `README.md`
  - CLI 導線と利用可能アルゴリズムの状態を同期する

## 実装時の注意

- 論文再現の対象は `CNav 自体` であり、ALAN の学習器ではない
- action 更新周期と simulation timestep は分けて持つ
- neighbor ranking は `goal への相対前後関係` を使うため、goal 未設定 scenario への対応方針を先に決める
- `v_intent` の broadcast は algorithm 内部状態で十分表現できるので、初回実装では network abstraction を導入しない
- first pass では論文既定 action set と固定 `T=2` を優先し、その後に一般化する

## 初期実装スコープ

初回実装では、論文再現と repo への組み込みを両立するため、次の範囲に限定する。

- action set は論文既定の 8 方向固定とする
- simulation horizon は `T = 2` とする
- `coordination_factor` は parameter 化するが、default は論文寄りの値を採用する
- constrained neighbor ranking を使う都合上、初回は `全 agent に goal_position 必須` とする
- 通信は network abstraction を導入せず、algorithm 内部の intended velocity cache で表現する

この初期実装で狙うのは、`CNav の action-selection 層が既存 ORCA 上で動く` ところまでである。  
action set の一般化、通信モデルの抽象化、広い scenario 再現は後続 issue に回す。

## 参考文献

- Godoy, J., Karamouzas, I., Guy, S. J., Gini, M. "Implicit Coordination in Crowded Multi-Agent Navigation." AAAI 2016. <https://motion.cs.umn.edu/r/CNAV/>
- Godoy, J., Guy, S. J., Gini, M., Karamouzas, I. "C-Nav: Distributed Coordination in Crowded Multi-Agent Navigation." Preprint for Robotics and Autonomous Systems, November 2020. <https://www-users.cse.umn.edu/~gini/publications/papers/Godoy2020.pdf>
- Godoy, J., Karamouzas, I., Guy, S. J., Gini, M. "Adaptive Learning for Multi-Agent Navigation." AAMAS 2015. <https://motion.cs.umn.edu/r/ALAN/ALAN.pdf>
