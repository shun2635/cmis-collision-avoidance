# Issue 0035: CNav forPaper の temporary goal / changeDest guidance を移植する

- ステータス: completed
- 優先度: high
- 関連文書:
  - [0027-cnav-my-style-validation.md](0027-cnav-my-style-validation.md)
  - [0033-cnav-mystyle-direct-port-scenarios.md](0033-cnav-mystyle-direct-port-scenarios.md)
  - [../specifications/cnav-mystyle-direct-port-scenarios.md](../specifications/cnav-mystyle-direct-port-scenarios.md)
  - [../specifications/cnav-legacy-parity-gap-review.md](../specifications/cnav-legacy-parity-gap-review.md)

## 背景

`cnav_forpaper_direct_port.yaml` により setup fidelity は上がったが、`forPaper.cpp` の本質的な guidance 差分である

- `calculateTemporaryGoals()` の A* cell guidance
- `changeDest()` の `dest1/dest2` 往復切替

は未再現のままである。  
この差分は obstacle / cell field 系の挙動に直接効く。

## 目的

`forPaper.cpp` の scenario-specific guidance を、Python 側で再現可能な形へ切り出し、direct-port scenario 上の挙動差分を縮める。

## スコープ

- cell field とセル中心座標の扱い整理
- temporary goal 更新ロジックの移植方針決定
- `dest1/dest2` 切替と往復走行の表現方法決定
- 必要なら scenario schema 拡張または code-generated setup 導線の追加
- tests / specs / README の同期

## 非スコープ

- 17 action set の導入
- legacy reward / politeness pipeline の移植
- C++ trace exporter

## 完了条件

- `forPaper` 系で temporary goal が更新される
- `dest1/dest2` の切替条件が定義されている
- direct-port scenario または code-generated setup で guidance を再現できる
- 関連 tests と docs が更新されている

## 想定成果物

- `src/cmis_ca/` 側の guidance 実装
- scenario / helper の更新
- `tests/` の更新
- `docs/specifications/` と `README.md` の更新

## 作業メモ

- CNav 本体と scenario guidance の責務を混ぜない
- 一般化が重すぎる場合は、まず `forPaper` 専用 helper として切り出す
- 既存 YAML schema で表現が難しければ、upstream regression と同様に code-generated setup を許容する

## 実施メモ

- `AgentConfig.goal_sequence` と `Scenario.navigation_grid` を追加し、goal cycle と cell guidance の入力面を作った
- `Simulator` に dynamic goal 更新を追加し、`forPaper` 用の temporary goal を preferred velocity へ反映できるようにした
- `scenarios/cnav_forpaper_direct_port.yaml` を guidance 付き direct-port scenario へ更新した
- `tests/core/test_simulation.py`、`tests/core/test_world.py`、`tests/io/test_scenario_loader.py` を追加 / 更新し、grid guidance と goal sequence を固定した
- `docs/specifications/cnav-mystyle-direct-port-scenarios.md`、`docs/specifications/cnav-legacy-parity-gap-review.md`、`README.md` を実装に合わせて更新した
