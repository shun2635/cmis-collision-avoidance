# CNav trace parity harness 仕様

## 1. 目的

この文書は、issue `0030` で追加した CNav trace parity harness の入出力を記録する。  
目的は、validation scenario 上で `step x agent x action` の比較材料を安定して取得できるようにすることである。

## 2. 実装位置

| 種別 | パス | 役割 |
| --- | --- | --- |
| trace model / runner | `src/cmis_ca/algorithms/cnav/trace.py` | CNav trace の dataclass、runner、JSONL writer |
| algorithm hook | `src/cmis_ca/algorithms/cnav/algorithm.py` | 各 step の action evaluation と出力速度を `latest_step_trace` として保持 |
| dump script | `scripts/dump_cnav_trace.py` | scenario 実行と JSONL dump |
| tests | `tests/algorithms/test_cnav_trace.py` | validation scenario を使った trace smoke |

## 3. trace の単位

trace は `step x agent` の flat record 列として保持する。  
各 record は次を持つ。

- `step_index`
- `global_time`
- `agent_index`
- `agent_name`
- `action_updated`
- `ranked_neighbors`
- `communicated_intended_velocity`
- `chosen_action_index`
- `chosen_intended_velocity`
- `output_velocity`
- `candidate_actions`

`candidate_actions` は、action 更新が走った step だけ埋まる。  
更新が走らない step では、`chosen_action_index` と `chosen_intended_velocity` は cache から継続し、`candidate_actions` は空列になる。

## 4. candidate action の列

各 candidate action は次を持つ。

- `action_index`
- `intended_velocity`
- `goal_progress_reward`
- `constrained_reduction_reward`
- `total_reward`

初期実装では論文既定の 8 action 固定なので、更新 step では `candidate_actions` の長さは `8` になる。

## 5. runner

`run_cnav_trace(scenario, algorithm, *, steps=None)` は:

1. `Simulator` を生成する
2. step ごとに `Simulator.step()` を呼ぶ
3. 各 step 後に `algorithm.latest_step_trace` を回収する
4. `SimulationResult` と `CNavTrace` を返す

`steps` を与えた場合は、その固定 step 数だけ回す。  
`steps` を省略した場合は scenario の停止条件に従う。

## 6. JSONL dump

`write_cnav_trace_jsonl(trace, path)` は、1 record 1 行の JSON Lines で書き出す。  
`scripts/dump_cnav_trace.py` はこの writer を呼ぶ最小の補助 script である。

比較用の named preset を script から選ぶため、`--profile {paper, legacy-forpaper-comparison}` を受け付ける。  
`legacy-forpaper-comparison` は、mainline default を変えずに `forPaper` 系と近い cadence / horizon / coordination factor で trace を取るための補助設定である。

実行例:

```bash
poetry run python scripts/dump_cnav_trace.py scenarios/cnav_queue_validation.yaml --steps 3
poetry run python scripts/dump_cnav_trace.py scenarios/cnav_queue_validation.yaml --steps 3 --profile legacy-forpaper-comparison
poetry run python scripts/dump_cnav_trace.py scenarios/cnav_crossing_validation.yaml --output /tmp/cnav-trace.jsonl
```

## 7. validation smoke

現時点の自動確認:

- `tests/algorithms/test_cnav_trace.py`
  - validation scenario ごとに record 数が取れること
  - 更新 step で 8 action 分の候補が取れること
  - carry-over step で `candidate_actions == ()` になること
  - JSONL writer が 1 record 1 行で出力すること

## 8. 制約

- まだ C++ 側 trace を自動取得して直接 diff するところまでは行っていない
- trace は CNav Python 実装の内部評価を出すものであり、legacy と一致する保証はまだない
- 17 action の MyStyle baseline を直接再現する trace ではなく、現行 Python 実装の 8 action trace である

この harness は issue `0031` の parity gap review の根拠データを作るための土台として使う。
