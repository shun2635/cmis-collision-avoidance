# Issue 0013: ORCA の agent 設定と simulator 振る舞いの差分を監査する

- ステータス: completed
- 優先度: high
- 関連文書:
  - [docs/architecture/orca-reproduction-roadmap.md](../architecture/orca-reproduction-roadmap.md)
  - [docs/algorithms/orca.md](../algorithms/orca.md)
  - [external/RVO2/src/RVOSimulator.cc](../../external/RVO2/src/RVOSimulator.cc)
  - [external/RVO2/src/Agent.cc](../../external/RVO2/src/Agent.cc)

## 背景

現在の `Simulator` と `AgentProfile` は研究室向けに単純化されている。  
ORCA の完全再現を優先するなら、upstream が前提にしている agent defaults、time step、状態更新、parameter の意味を再監査する必要がある。

## 目的

現行 Python 実装と upstream の agent / simulator semantics の差分を洗い出し、必要な項目を core 側へ反映する。

## スコープ

- upstream の agent defaults を整理する
- `AgentProfile` `AgentConfig` `Scenario` `Simulator` の差分を監査する
- Python 側へ導入すべきパラメータを決める
- 不要な簡略化と必要な互換層を分けて記録する
- 回帰テストと仕様書を更新する

## 非スコープ

- obstacle graph の移植
- regression suite の拡張

## 完了条件

- upstream との差分表が docs にある
- 導入すべき agent / simulator parameter が Python 実装へ反映されている
- ORCA 実装が参照する parameter の意味が docs 上で固定されている

## 想定成果物

- `src/cmis_ca/core/agent.py`
- `src/cmis_ca/core/world.py`
- `src/cmis_ca/core/simulation.py`
- `docs/specifications/`

## 依存関係

- [0012-orca-goal-model-and-pref-velocity-alignment.md](0012-orca-goal-model-and-pref-velocity-alignment.md): completed

## 作業メモ

- `AgentProfile` に upstream 寄りの per-agent parameter を追加した
- `ORCAParameters` は agent profile を上書きする optional override に変更した
- `Simulator.global_time` と `WorldSnapshot.global_time` を追加した
- scenario loader が navigation parameter を profile へ読み込むようにした
- 差分表は `docs/specifications/orca-agent-simulator-parity.md` に整理した
