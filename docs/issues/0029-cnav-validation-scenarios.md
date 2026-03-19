# Issue 0029: CNav 検証用 scenario を追加する

- ステータス: completed
- 優先度: high
- 関連文書:
  - [docs/issues/0027-cnav-my-style-validation.md](0027-cnav-my-style-validation.md)
  - [docs/issues/0028-cnav-my-style-baseline-audit.md](0028-cnav-my-style-baseline-audit.md)
  - [docs/specifications/cnav-initial-implementation.md](../specifications/cnav-initial-implementation.md)

## 背景

現状の `scenarios/cnav_queue.yaml` は smoke 用としては十分だが、legacy parity 検証には用途が足りない。  
検証では「どの分岐を見たい scenario か」が明確である必要がある。

## 目的

baseline 比較に使う最小 scenario 群を repo に追加し、scenario ごとの役割と観測指標を固定する。

## スコープ

- `queue`
- `head-on`
- `crossing`
- `obstacle`

上記の validation scenario 設計と追加。

## 非スコープ

- 大規模 crowd scenario の完全移植
- trace dump 実装
- parity 判定そのもの

## 完了条件

- validation 用 scenario が `scenarios/` に追加されている
- 各 scenario の目的が issue または仕様書で説明されている
- 停止条件と比較指標が固定されている
- smoke 用 scenario と validation 用 scenario の役割が分離されている

## 想定成果物

- `scenarios/cnav_queue_validation.yaml`
- `scenarios/cnav_head_on_validation.yaml`
- `scenarios/cnav_crossing_validation.yaml`
- `scenarios/cnav_obstacle_validation.yaml`

## 作業メモ

- 初手は YAML で持つ
- 1 scenario 1 観測目的を基本にする
- 大規模 case が必要になったら別 issue で code-generated setup を検討する

## 実施メモ

- `scenarios/cnav_queue_validation.yaml` を追加し、狭隘 gap を持つ 2-agent counterflow の validation case を固定した
- `scenarios/cnav_head_on_validation.yaml` を追加し、2 agent 対向ケースを固定した
- `scenarios/cnav_crossing_validation.yaml` を追加し、4-way crossing の validation case を固定した
- `scenarios/cnav_obstacle_validation.yaml` を追加し、障害物つき validation case を固定した
- scenario の目的、停止条件、観測指標を `docs/specifications/cnav-validation-scenarios.md` に記録した
- smoke 用 `scenarios/cnav_queue.yaml` と validation 用 scenario 群の役割分離を docs に反映した
