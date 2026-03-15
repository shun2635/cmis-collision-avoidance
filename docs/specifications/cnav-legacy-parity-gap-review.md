# CNav legacy parity gap review

## 1. 目的

この文書は、issue `0031` で `external/CNav_MyStyle` と現行 Python 実装の差分を棚卸しし、後続修正で何を採用するかの判断基準を固定する。  
対象は `2026-03-14` 時点の

- legacy baseline: `external/CNav_MyStyle/simulations/forPaper.cpp`
- secondary baseline: `external/CNav_MyStyle/simulations/crowdForPaper.cpp`
- Python 実装: `src/cmis_ca/algorithms/cnav/`

である。

## 2. 前提

- legacy baseline の固定は `docs/specifications/cnav-my-style-baseline-audit.md` に従う
- Python trace harness の入出力は `docs/specifications/cnav-trace-parity-harness.md` に従う
- この review は「legacy に寄せるべきか」を判断するためのものであり、「legacy と違うから直す」を前提にしない

## 3. 現時点の観測

### 3.1 Python trace の初期観測

validation scenario で現行 Python trace を見ると、最初の更新 step では次の傾向がある。

| scenario | 観測 |
| --- | --- |
| `cnav_queue_validation` | rear は `action 1`、front は `action 0` |
| `cnav_head_on_validation` | 両 agent とも `action 1` |
| `cnav_crossing_validation` | 全 agent が `action 1` |
| `cnav_obstacle_validation` | 両 agent とも `action 0` |

これは現行 Python 実装が trace を安定して出せることの確認にはなるが、まだ C++ 側自動 trace と直接 diff した結果ではない。

### 3.2 review の制約

- review 実施時点では C++ 側 trace の自動抽出は未実装だった
- その後 issue `0034` で初期の legacy trace export / diff workflow は追加されたが、`forPaper.cpp` の decision timing 差分解釈は依然必要である
- validation scenario は baseline driver の簡略化であり、まだ direct port ではない
- よって本 review は、まず `構造差分` と `既知の挙動差分候補` を分類するものとする

## 4. 差分一覧

| 項目 | legacy baseline | 現行 Python | 影響 | 判定 | 優先度 |
| --- | --- | --- | --- | --- | --- |
| action set cardinality / speed | `17` action。`1.5 x 8`、`0.75 x 8` または `0.1 x 8`、`0.0 x 1` | 論文寄り `8` action、speed 固定 `1.5` | chosen action と reward landscape が根本的に変わる | legacy-only heuristic | medium |
| action update cadence | `timeStep > 1` 以降は毎 step 再評価 | `action_update_interval = 0.2` の固定周期 | trace の比較時刻がずれる | intentional difference | medium |
| coordination factor default | `0.9` | `0.8` | reward weighting がずれる | intentional difference | low |
| simulation horizon | `3` | `2` | reward accumulation の長さがずれる | intentional difference | medium |
| reward / politeness pipeline | `sameWeight`、`counterDirection`、`slowMove`、neighbor 上限、`endOfPlan` 平均を含む | 論文式の `R_ga` / `R_ca` を正規化して使用 | chosen action が変わりやすい | legacy-only heuristic | high |
| simulate / think neighbor caps | `maxNumSimulateNeighbors`、`maxNumThinkNeighbors` を使用 | top-k constrained neighbors のみ | ranking 後の評価対象がずれる | legacy-only heuristic | medium |
| temporary goal model | `forPaper` 系は temporary goal を計算して preferred velocity を更新 | static `goal_position` への直線追従 | obstacle / cell field 系で挙動が大きく変わる | legacy-only heuristic | high |
| scenario fidelity | baseline は 12 agent cell field / 50 agent corridor crowd | 2-4 agent の簡略 validation scenario | direct parity claim が弱い | fix | high |
| trace comparison strength | C++ 側 action / reward trace を自動収集していない | Python 側 JSONL trace は取得可能 | 数値 parity をまだ判定できない | fix | high |

## 5. 判定理由

### 5.1 `fix`

#### 1. scenario fidelity

minimal validation scenario は分岐確認には十分だが、`forPaper.cpp` や `crowdForPaper.cpp` の direct parity とは言えない。  
そのため、`2026-03-14` 付で `scenarios/cnav_forpaper_direct_port.yaml` と `scenarios/cnav_crowd_forpaper_direct_port.yaml` を追加し、setup fidelity を 1 段引き上げた。  
ただし `changeDest()` と temporary goal は未再現なので、scenario fidelity gap が完全に消えたわけではない。

#### 2. trace comparison strength

Python 側 trace は整い、issue `0034` で legacy C++ 側の初期 trace export も追加された。  
ただし `forPaper.cpp` は action 決定と通信の timing が Python と 1 step ずれるため、まだ「どの step で何が違うか」を完全に言い切れる段階ではない。  
この差分は、tooling gap から `timing-aware comparison gap` へ縮退したとみなす。

### 5.2 `intentional difference`

#### 1. update cadence

現行 Python は、repo 全体の方針どおり `論文寄りの固定周期` を採っている。  
`forPaper.cpp` の毎 step 更新は current MyStyle baseline の特徴だが、repo の mainline default を直ちに変える根拠にはならない。

#### 2. coordination factor / simulation horizon

`γ = 0.8` と `T = 2` は Python 側 docs と論文整理に整合している。  
legacy baseline の `0.9` / `3` は比較対象としては重要だが、mainline default を変える理由ではない。

### 5.3 `legacy-only heuristic`

#### 1. 17-action set

17 action は `forPaper` / `crowdForPaper` 固有で、論文 first-pass の 8 action とは別物である。  
現時点では、これを mainline へ入れるよりも `legacy profile` として切り出す方が自然である。

#### 2. reward / politeness pipeline

legacy baseline の `SimulateVelocity(...)` は、論文の核となる reward に加えて

- `sameWeight`
- `counterDirection`
- `slowMove`
- neighbor 上限
- `endOfPlan - 1` による平均化

を混ぜている。  
これは current MyStyle 実験系のヒューリスティクスとしては重要だが、そのまま mainline の仕様に昇格させる根拠はまだ足りない。

#### 3. temporary goal model

`forPaper.cpp` の temporary goal は、CNav 本体というより scenario-specific path guidance に近い。  
現行 Python CNav の「coordination layer」とは責務が違うため、mainline の parity gap と scenario guidance gap を分けて扱うべきである。

## 6. 後続 issue への指示

### 6.1 issue `0032` で優先して扱うもの

1. Python 側に `legacy parity profile` の導入余地を持たせるかの判断
2. trace / scenario を docs と矛盾なく参照できるよう同期する

`0032` では、mainline default は維持したまま、`legacy-forpaper-comparison` preset を追加して比較用 cadence / horizon / coordination factor を切り替えられるようにする方針を採る。

### 6.2 別 issue で切るべきもの

次は `0032` に入れず、必要なら別 issue で切る。

- C++ 側 trace exporter
- `forPaper` 系 scenario の direct port から先の temporary goal 移植
- 17 action と mixed-speed action set の本格導入
- temporary goal / cell guidance の移植

## 7. 結論

現時点で最も重要なのは、`legacy baseline と Python mainline は意図的に同一ではない` ことを docs 上で明示したうえで、  
direct parity を主張するために不足している `scenario fidelity` と `trace comparison strength` を先に埋めることである。

したがって、後続の修正は

1. direct parity を強めるための tooling / scenario gap
2. その後で必要なら legacy profile を追加するかの判断

の順で進める。  
17 action や temporary goal などの legacy 振る舞いを、現時点で mainline default に寄せる必要はない。
