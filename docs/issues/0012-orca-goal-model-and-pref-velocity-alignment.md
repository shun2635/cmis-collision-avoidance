# Issue 0012: ORCA の goal モデルと preferred velocity 更新を一般化する

- ステータス: open
- 優先度: high
- 関連文書:
  - [docs/architecture/orca-reproduction-roadmap.md](../architecture/orca-reproduction-roadmap.md)
  - [docs/specifications/upstream-circle-regression.md](../specifications/upstream-circle-regression.md)
  - [external/RVO2/examples/Circle.cc](../../external/RVO2/examples/Circle.cc)

## 背景

現在の `Circle` 回帰では、goal と preferred velocity の更新を専用 helper に閉じ込めている。  
しかし upstream のサンプルや今後の ORCA 再現では、goal を scenario レベルで表現し、各 step で preferred velocity を更新する導線が必要になる。

## 目的

goal / destination を scenario schema と simulator の正規機能として導入し、upstream 由来の preferred velocity 更新を専用 helper なしで表現できるようにする。

## スコープ

- `Scenario` に goal または destination の表現を追加する
- `AgentConfig` に必要な静的 goal 情報を追加する
- `Simulator` または ORCA 実行導線に preferred velocity 更新 hook を追加する
- `Circle` 回帰 helper が専用ロジックに依存しない形へ移行する
- scenario schema と tests を更新する

## 非スコープ

- obstacle topology の再設計
- ORCA obstacle line の厳密移植
- proxemic / cnav 用 goal モデルの一般化

## 完了条件

- scenario file で goal を記述できる
- `Circle` 相当ケースで goal 更新が正規機能として動く
- dedicated helper なしでも upstream 的な preferred velocity 更新が表現できる
- 仕様書と API 文書が同期されている

## 想定成果物

- `src/cmis_ca/core/agent.py`
- `src/cmis_ca/core/simulation.py`
- `src/cmis_ca/io/scenario_loader.py`
- `scenarios/upstream_circle.yaml`
- `tests/`

## 依存関係

- [0010-upstream-circle-scenario-regression.md](0010-upstream-circle-scenario-regression.md): completed
