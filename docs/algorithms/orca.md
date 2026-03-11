# ORCA

## 位置付け

ORCA は、このリポジトリで最初に扱う基準アルゴリズムである。  
目的は ORCA そのものを単独実装することではなく、ORCA を起点にして proxemic や CNav などを同一基盤で比較できる構造を作ることにある。

この文書では、ORCA の理論を網羅的に解説するのではなく、以下を固定する。

- ORCA の 1 ステップ処理を、研究室内で読める粒度に分解する
- `src/cmis_ca/core/` に入れるものを決める
- `src/cmis_ca/algorithms/orca/` に閉じ込めるものを決める
- 将来の proxemic / CNav 追加時に壊れにくい境界を決める

## upstream との関係

upstream 参照コードは `external/RVO2` に置いてあり、ORCA の主な処理は以下に現れている。

- `external/RVO2/src/RVOSimulator.cc`
  - シミュレータ全体のステップ進行
- `external/RVO2/src/Agent.cc`
  - 近傍取得
  - ORCA 制約生成
  - 線形計画による速度決定
  - 位置と速度の更新

本リポジトリでは、upstream のクラス分割をそのまま模倣しない。  
代わりに、`共通コア` と `アルゴリズム固有差分` の責務単位に再整理する。

## ORCA の 1 ステップ処理

ORCA の 1 ステップは、研究室実装では次の 6 段階に分けて扱う。

1. 現在のワールド状態から、各エージェントの局所スナップショットを作る
2. 障害物近傍と他エージェント近傍を取得する
3. 障害物に由来する ORCA 制約を生成する
4. エージェント間に由来する ORCA 制約を生成する
5. 希望速度に最も近い実行可能速度を、速度上限制約付きの 2 次元線形計画で選ぶ
6. 決定した速度で位置と速度を更新する

upstream ではこの流れが `computeNeighbors` `computeNewVelocity` `update` にまとまっているが、研究室実装ではそれぞれをさらに責務で分ける。

## ORCA の処理分解

### 1. ワールド状態の準備

入力として必要なのは以下。

- 各エージェントの位置
- 各エージェントの現在速度
- 各エージェントの希望速度
- 各エージェントの半径
- 各エージェントの最大速度
- 障害物形状
- タイムステップ

この時点では、まだ ORCA 固有の意味付けは薄い。  
したがって、ワールド状態やエージェント状態は共通コアに置く。

### 2. 近傍探索

ORCA は、全エージェントのうち必要な相手だけを見て制約を作る。  
そのため、近傍探索機構自体は共通コアに置く。

ただし、どの距離まで探すか、何体まで採用するかは ORCA のパラメータに依存する。  
したがって、近傍探索エンジンと、ORCA の問い合わせ方は分離する。

現時点の参照実装では、`core/neighbor_search.py` が以下を担当する。

- 問い合わせ対象エージェントを `agent_index` で特定する
- `neighbor_dist` 以内のエージェントを距離順で返す
- `max_neighbors` でエージェント数だけを制限する
- `neighbor_dist` 以内の障害物線分を、点と線分の最短距離で距離順に返す

一方で、どの距離閾値を使うか、何体まで見るか、返ってきた近傍を制約生成でどう解釈するかは ORCA 側に残す。

### 3. 障害物 ORCA 制約の生成

障害物に対する ORCA 制約では、以下の判断が入る。

- 障害物辺がすでに既存制約で覆われているか
- 頂点衝突か、辺衝突か
- 凸頂点か非凸頂点か
- cut-off line と左右の leg のどこへ射影するか

ここは ORCA の幾何意味が強く、proxemic や CNav と共有しにくい。  
したがって、制約生成ロジック自体は ORCA 固有差分として扱う。

現時点の Python 実装では、障害物は upstream の linked obstacle graph ではなく `ObstacleSegment` 単体で表現している。  
そのため、障害物 ORCA 制約は「線分上の最近点を静的障害物点としてみなす closest-point 近似」で実装している。  
これは upstream と完全一致ではないが、現リポジトリの障害物モデルに対する基準実装として位置付ける。

### 4. エージェント間 ORCA 制約の生成

エージェント間制約では、以下の判断が入る。

- 相対位置と相対速度を使う
- 衝突中か非衝突かで場合分けする
- cut-off circle と左右 leg のどちらへ射影するかを決める
- 回避責任を半分ずつ負担する形で補正量を作る

この「半分ずつ責任を負う」という意味付けは ORCA の中心概念であり、共通コアへ入れるべきではない。  
したがって、ここも ORCA 固有差分として扱う。

現時点の Python 実装では、この層は upstream の agent-agent ORCA line 生成をベースに移植済みである。

### 5. 実行可能速度の決定

ORCA では、生成した半平面制約群と最大速度円の中で、希望速度に最も近い速度を求める。  
この段階は次の 2 層に分けて扱う。

- 制約の意味付け
  - ORCA 固有
- 制約を満たす点を 2 次元で探索するソルバ
  - 共通コア候補

upstream の `linearProgram1` `linearProgram2` `linearProgram3` は ORCA 実装の中に埋まっているが、研究室実装では `半平面 + 円制約に対する汎用 2D ソルバ` として切り出す。

ただし、ソルバへ渡す目的関数の選び方はアルゴリズム依存である。  
ORCA では「希望速度に最も近い点」を選ぶが、将来のアルゴリズムでは別の評価関数を使う可能性がある。

現時点では、この層は `core/solver.py` に `solve_linear_constraints()` と `choose_preferred_velocity()` として実装済みである。  
また、ORCA 側の agent-agent / obstacle 制約生成も `algorithms/orca/constraints.py` に実装済みであり、`ORCAAlgorithm.step()` では obstacle line 数を `protected_constraint_count` として solver へ渡す統合まで完了している。

### 6. 状態更新

速度が決まった後の処理は単純である。

- 新速度を現在速度へ反映する
- `position += velocity * time_step` で位置を進める

この更新則は ORCA 固有ではないため、共通コアに置く。

## データ構造案

ORCA を実装するうえで必要な概念を、研究室実装では以下のように分ける。

- `Vector2`
  - 2 次元ベクトル
- `AgentState`
  - 現在位置、現在速度、希望速度
- `AgentProfile`
  - 半径、最大速度、その他の固定パラメータ
- `WorldSnapshot`
  - 時刻、エージェント一覧、障害物一覧、共通設定
- `NeighborSet`
  - あるエージェントから見た近傍情報
- `AgentNeighbor` / `ObstacleNeighbor`
  - 距離付きの近傍結果
- `LineConstraint`
  - 2 次元速度空間における半平面制約
- `AgentCommand`
  - そのステップで採用する速度指令
- `OrcaParameters`
  - ORCA 固有パラメータ

ここで重要なのは、`LineConstraint` は ORCA line という名前のままコアへ持ち込まないことだ。  
コア側では中立的な名前にし、ORCA はその一利用者として扱う。

## 共通コアと ORCA 固有差分の分類

| 要素 | 配置先 | 理由 |
| --- | --- | --- |
| 2 次元ベクトル演算 | `core/geometry.py` | どのアルゴリズムでも使う |
| 外積、内積、正規化、距離計算 | `core/geometry.py` | 幾何の基礎であり ORCA 専用ではない |
| エージェント状態 | `core/state.py` | 位置と速度は共通概念 |
| エージェントの半径、最大速度 | `core/agent.py` | 物理的属性として共通に扱える |
| 希望速度 | `core/state.py` | ORCA だけでなく他アルゴリズムでも使う可能性が高い |
| 障害物表現 | `core/world.py` | シナリオ共有のため共通化が必要 |
| ワールドスナップショット | `core/world.py` | 全アルゴリズム共通の入力面になる |
| 近傍探索エンジン | `core/neighbor_search.py` | 探索機構自体は再利用できる |
| 半平面制約の中立表現 | `core/constraints.py` | ORCA 以外でも使える可能性がある |
| 円制約付き 2D ソルバ | `core/solver.py` | ORCA 外でも再利用可能な補助計算 |
| シミュレーション更新ループ | `core/simulation.py` | アルゴリズム非依存の実行基盤 |
| ORCA の近傍採用方針 | `algorithms/orca/parameters.py` | 探索機構ではなく ORCA の判断基準だから |
| ORCA の `time_horizon` | `algorithms/orca/parameters.py` | ORCA 固有の意味を持つ |
| ORCA の `time_horizon_obst` | `algorithms/orca/parameters.py` | ORCA 固有の意味を持つ |
| 障害物 ORCA 制約生成 | `algorithms/orca/constraints.py` | ORCA 特有の幾何と場合分けを持つ |
| エージェント間 ORCA 制約生成 | `algorithms/orca/constraints.py` | reciprocal avoidance の意味付けが ORCA 固有 |
| ORCA 制約群の組み立て | `algorithms/orca/algorithm.py` | ORCA の実行戦略に属する |
| 希望速度を目的にした ORCA の最適化呼び出し | `algorithms/orca/algorithm.py` | ソルバ利用方法は ORCA 固有 |
| ORCA デバッグ情報 | `algorithms/orca/` | 制約本数や ORCA line の可視化は ORCA 固有 |

## `src/cmis_ca/` へのマッピング案

### `core/`

- `core/geometry.py`
  - `Vector2`
  - 幾何補助関数
- `core/agent.py`
  - エージェントの固定属性
- `core/state.py`
  - 状態、コマンド、スナップショット要素
- `core/world.py`
  - ワールド、障害物、シナリオ
- `core/neighbor_search.py`
  - 近傍探索インターフェースと実装
- `core/constraints.py`
  - 半平面制約の中立表現
- `core/solver.py`
  - 制約付き速度選択の汎用ソルバ
- `core/simulation.py`
  - ステップ進行と状態更新

### `algorithms/orca/`

- `algorithms/orca/parameters.py`
  - `neighbor_dist`
  - `max_neighbors`
  - `time_horizon`
  - `time_horizon_obst`
- `algorithms/orca/constraints.py`
  - 障害物 ORCA 制約生成
  - エージェント間 ORCA 制約生成
- `algorithms/orca/algorithm.py`
  - ORCA の 1 ステップ判断
  - ソルバ呼び出し
  - 結果を `AgentCommand` へ変換

## ORCA 境界の判断基準

以下の基準で `core` と `algorithms/orca` を切る。

### `core` に入れる条件

- ORCA 以外でも意味が変わらず使える
- 中立的な名前で定義できる
- 将来の proxemic / CNav でも入力や出力として使える
- 数学的には ORCA の中で使われていても、実装上は汎用部品として切り出せる

### `algorithms/orca` に入れる条件

- ORCA 特有の概念名を持つ
- reciprocal avoidance の前提に依存する
- `time_horizon` のように ORCA の時間解釈を含む
- 他アルゴリズムで同じ計算をしても意味が変わる

## ORCA で共通コアへ入れすぎないための注意

ORCA を最初に実装すると、使っている概念をそのまま全部 `core/` に押し込みたくなる。  
これは避けるべきである。

特に以下は、ORCA では重要でもコアへ入れすぎない。

- `OrcaLine` という名前のままの制約型
- ORCA 固有パラメータを含んだエージェントクラス
- ORCA 用の近傍選択ロジックを埋め込んだ近傍探索器
- ORCA の目的関数を前提にしたシミュレータ本体

コアは「多アルゴリズム基盤」であり、「ORCA を一般化したもの」ではない。

## 将来の proxemic / CNav 追加時に衝突しそうな論点

### 近傍探索の意味

ORCA は衝突回避のための近傍を見るが、proxemic では社会的距離を考えるため、同じ近傍でも意味が変わる。  
したがって、探索エンジンは共通化してよいが、距離閾値や採用規則はアルゴリズム側へ残す。

### 目的関数の違い

ORCA は希望速度への近さを重視する。  
proxemic や CNav では、快適性、経路意図、コスト最小化など別の目的関数を使う可能性が高い。  
そのため、ソルバは共通化できても、最適化問題の立て方はアルゴリズム側に残す。

### 制約表現の有無

ORCA は半平面制約が中心だが、他アルゴリズムでは制約というよりコスト場や経路候補評価になるかもしれない。  
そのため、`Simulator` がすべてのアルゴリズムに制約列を要求する設計にはしない。

### 社会属性の扱い

proxemic では個人距離や関係性などの属性を持つ可能性がある。  
これは将来的に `core` 側のエージェントメタデータとして入る可能性はあるが、意味付けはアルゴリズム側に残すべきである。

## この文書を前提にした実装方針

Python 実装へ進む際は、まず ORCA を以下の順で作る。

1. `core/geometry.py` `core/state.py` `core/world.py`
2. `core/neighbor_search.py`
3. `core/constraints.py` `core/solver.py`
4. `algorithms/orca/parameters.py`
5. `algorithms/orca/constraints.py`
6. `algorithms/orca/algorithm.py`
7. `core/simulation.py` への統合

この順序により、ORCA を実装しながらも、後続アルゴリズムを入れる余地を残せる。
