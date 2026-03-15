# Issue 0034: CNav legacy C++ trace export と diff workflow を追加する

- ステータス: completed
- 優先度: high
- 関連文書:
  - [0027-cnav-my-style-validation.md](0027-cnav-my-style-validation.md)
  - [0030-cnav-trace-parity-harness.md](0030-cnav-trace-parity-harness.md)
  - [0031-cnav-legacy-parity-gap-review.md](0031-cnav-legacy-parity-gap-review.md)
  - [../specifications/cnav-trace-parity-harness.md](../specifications/cnav-trace-parity-harness.md)

## 背景

Python 側は JSONL trace を出せるようになったが、legacy C++ 側はまだ action / reward / velocity の自動 trace export を持っていない。  
このため direct-port scenario を追加しても、差分を `step x agent x action` で数値比較できない。

## 目的

`external/CNav_MyStyle` 側から、Python trace と比較可能な legacy trace を安定して取得し、半自動 diff できる導線を作る。

## スコープ

- `forPaper.cpp` と `crowdForPaper.cpp` の trace 出力点の特定
- action / reward / intended velocity / output velocity を吐く exporter の追加
- Python trace JSONL と見比べられる field 定義の固定
- repo から実行できる比較用 script または手順の追加
- 仕様書と README の同期

## 非スコープ

- legacy algorithm 自体の振る舞い変更
- Python 側の reward model 修正
- GUI 比較

## 完了条件

- legacy C++ 側で trace をファイル出力できる
- Python 側 trace と key が対応づく
- direct-port scenario の少なくとも 1 ケースで diff 手順が再現可能
- 関連 docs が更新されている

## 想定成果物

- `external/CNav_MyStyle/` 配下の最小 trace 出力変更
- compare 用 script または手順書
- `docs/specifications/` の更新
- `README.md` の更新

## 作業メモ

- Python 側 `candidate_actions` と完全一致しない field は、対応表を文書で明示する
- 初手は `forPaper` を優先し、`crowdForPaper` は secondary smoke に留めてもよい
- legacy ソースの変更は最小限に抑え、比較専用の patch で閉じる

## 実施メモ

- `external/CNav_MyStyle/simulations/forPaper.cpp` と `crowdForPaper.cpp` に env-driven trace export hook を追加した
- `external/CNav_MyStyle/simulations/legacy_trace_export.h` を追加し、JSONL writer と trace env 読み込みを共通化した
- `scripts/dump_cnav_legacy_trace.py` を追加し、repo 内のパスで legacy driver を build / run できるようにした
- `scripts/compare_cnav_trace_jsonl.py` を追加し、Python trace JSONL と legacy trace JSONL の共通 field diff を出せるようにした
- `docs/specifications/cnav-trace-parity-harness.md` と `README.md` を legacy trace workflow に合わせて同期した
- `forPaper.cpp` の action evaluation timing は Python と 1 step ずれるため、legacy trace では `next_action_index` / `next_chosen_intended_velocity` を extra field として保持する
