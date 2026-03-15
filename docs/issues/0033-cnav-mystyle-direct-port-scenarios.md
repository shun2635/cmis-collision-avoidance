# Issue 0033: CNav MyStyle の direct-port scenario を追加する

- ステータス: completed
- 優先度: medium
- 関連文書:
  - [0027-cnav-my-style-validation.md](0027-cnav-my-style-validation.md)
  - [0031-cnav-legacy-parity-gap-review.md](0031-cnav-legacy-parity-gap-review.md)
  - [../specifications/cnav-mystyle-direct-port-scenarios.md](../specifications/cnav-mystyle-direct-port-scenarios.md)

## 背景

issue `0031` で `scenario fidelity` は high priority の fix と分類された。  
既存の validation scenario は分岐確認には有効だが、`forPaper.cpp` と `crowdForPaper.cpp` の setup を直接比較するには簡略化が大きい。

## 目的

`external/CNav_MyStyle` の primary / secondary baseline から、現行 YAML schema で再現可能な setup を direct-port scenario として追加する。

## スコープ

- `forPaper.cpp` 由来 scenario の追加
- `crowdForPaper.cpp` 由来 scenario の追加
- 関連仕様書、README、tests の同期

## 非スコープ

- `temporary goal` の移植
- `changeDest()` の往復 goal 切替
- legacy 17 action / reward pipeline の移植

## 完了条件

- direct-port scenario が `scenarios/` に追加されている
- loader test で読み込める
- 仕様書と README が導線を持つ
- 未再現項目が明記されている

## 実施メモ

- `scenarios/cnav_forpaper_direct_port.yaml` を追加し、12 agent + 36 obstacle cell + first-leg goal を固定した
- `scenarios/cnav_crowd_forpaper_direct_port.yaml` を追加し、50 agent corridor crowd を固定した
- `docs/specifications/cnav-mystyle-direct-port-scenarios.md` に fidelity と limitation を記録した
- `tests/io/test_scenario_loader.py` と `tests/algorithms/test_cnav_trace.py` に smoke を追加した
