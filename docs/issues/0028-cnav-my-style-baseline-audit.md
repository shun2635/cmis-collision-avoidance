# Issue 0028: `external/CNav_MyStyle` の baseline を監査して固定する

- ステータス: completed
- 優先度: high
- 関連文書:
  - [docs/issues/0027-cnav-my-style-validation.md](0027-cnav-my-style-validation.md)
  - [docs/algorithms/cnav.md](../algorithms/cnav.md)
  - [docs/specifications/cnav-initial-implementation.md](../specifications/cnav-initial-implementation.md)

## 背景

`external/CNav_MyStyle` には `forPaper.cpp`、`crowdForPaper.cpp`、`circle.cpp`、`crowd.cpp` など複数の driver があり、更新規則やフラグが一致していない。  
この状態で parity を取りに行くと、「何と比較しているのか」がぶれる。

## 目的

比較に使う legacy baseline を固定し、driver ごとの差分を表で整理する。

## スコープ

- `external/CNav_MyStyle` の主要 driver 読解
- action 更新規則、`coordFactor`、`allNeigh`、neighbor 上限、time horizon の整理
- paper-facing baseline と my-style baseline の切り分け
- baseline 固定表の作成

## 非スコープ

- Python 実装の修正
- scenario 追加
- trace dump 実装

## 完了条件

- 比較対象の primary baseline が 1 つ以上決まっている
- secondary baseline の用途が決まっている
- driver ごとの差分表が docs に残っている
- 後続 issue が baseline 前提で進められる

## 想定成果物

- baseline 比較メモ
- driver / parameter の固定表
- 必要なら `docs/algorithms/cnav.md` か `docs/specifications/` への反映

## 作業メモ

- `forPaper` 系を優先候補にしつつ、`circle.cpp` の確率更新は別系統として扱う
- `paper baseline` と `legacy experimental baseline` を混同しない

## 実施メモ

- `external/CNav_MyStyle/simulations/forPaper.cpp` を primary baseline に固定した
- `external/CNav_MyStyle/simulations/crowdForPaper.cpp` を secondary baseline に固定した
- `external/CNav_MyStyle/examples/circle.cpp` は確率更新を持つ旧系統として、legacy experimental baseline に分離した
- `simulationDefs.h` のグローバル既定値は driver の `main()` / `mainLoop(...)` によって上書きされるため、baseline 判定には使わない方針を固定した
- 監査結果を `docs/specifications/cnav-my-style-baseline-audit.md` と `docs/algorithms/cnav.md` に反映した
