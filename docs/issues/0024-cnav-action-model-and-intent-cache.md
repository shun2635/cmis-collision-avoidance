# Issue 0024: CNav の action model と intended velocity cache を追加する

- ステータス: completed
- 優先度: high
- 関連文書:
  - [docs/algorithms/cnav.md](../algorithms/cnav.md)
  - [docs/issues/0022-cnav-initial-implementation.md](0022-cnav-initial-implementation.md)

## 背景

CNav の最小構成には、論文既定の action set、更新周期、agent ごとの `v_intent` 保持が必要である。  
これらは ORCA には存在しないため、`algorithms/cnav/` の独自状態として導入する必要がある。

## 目的

CNav の parameter、action set 生成、intended velocity cache、algorithm 雛形を追加し、action-selection を載せる足場を作る。

## スコープ

- `algorithms/cnav/parameters.py`
- `algorithms/cnav/actions.py`
- `algorithms/cnav/algorithm.py` の最小雛形
- agent ごとの intended velocity cache
- action 更新周期の保持

## 非スコープ

- constrained neighbor ranking
- SimMotion と reward 計算
- CLI / scenario / tests の最終導線

## 完了条件

- 論文既定の 8 action を goal 方向から生成できる
- algorithm 内部で agent ごとの `v_intent` を保持できる
- action 更新タイミングを判定できる
- 最小の `CNavAlgorithm` が ORCA helper を呼べる土台を持つ

## 想定成果物

- `src/cmis_ca/algorithms/cnav/parameters.py`
- `src/cmis_ca/algorithms/cnav/actions.py`
- `src/cmis_ca/algorithms/cnav/algorithm.py`

## 作業メモ

- 初期実装では `T=2`、8 action、goal 必須を前提にしてよい
- 通信は cache で表現し、外部通信 abstraction は導入しない

## 実施メモ

- `src/cmis_ca/algorithms/cnav/` を追加し、`CNavParameters`、`build_default_action_set(...)`、`CNavAlgorithm` の雛形を実装した
- `CNavAlgorithm` は `goal-oriented baseline` から 8 action を生成し、現段階では先頭 action を intended velocity として cache する
- action 更新周期は `global_time` と `action_update_interval` で判定する
- 最小 test は `tests/algorithms/test_cnav.py` に追加した
