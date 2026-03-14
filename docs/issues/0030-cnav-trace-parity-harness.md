# Issue 0030: CNav の trace parity harness を追加する

- ステータス: todo
- 優先度: high
- 関連文書:
  - [docs/issues/0027-cnav-my-style-validation.md](0027-cnav-my-style-validation.md)
  - [docs/issues/0028-cnav-my-style-baseline-audit.md](0028-cnav-my-style-baseline-audit.md)
  - [docs/issues/0029-cnav-validation-scenarios.md](0029-cnav-validation-scenarios.md)

## 背景

終端の goal 到達時間や collision 数だけでは、CNav 本体の差分を特定しにくい。  
action、reward、velocity を step 単位で並べて見られる比較基盤が必要である。

## 目的

Python 実装と `external/CNav_MyStyle` の trace を比較できる harness を追加し、unit / trace / scenario の中間層を埋める。

## スコープ

- Python 側 trace dump の追加
- 比較用 script または test helper の追加
- action index、reward、intended velocity、ORCA 後 velocity の出力
- validation scenario を使った parity smoke

## 非スコープ

- 差分の最終判断
- 大規模 benchmark
- 可視化 UI

## 完了条件

- Python 側で CNav trace を安定して取得できる
- validation scenario 上で trace 比較を自動または半自動で回せる
- parity 判定に必要な列が揃っている
- 後続の gap review が trace を根拠に書ける

## 想定成果物

- `tests/algorithms/` 配下の parity helper / tests
- `scripts/` 配下の比較用補助スクリプト
- trace 出力仕様メモ

## 作業メモ

- まずは CSV か JSON Lines の簡素な出力で十分
- `step x agent x action` を追えない設計にはしない
