# ORCA agent / solver parity

## 1. 目的

この文書は、upstream `RVO2` の agent-agent ORCA line と `linearProgram1/2/3` に対して、現行 Python 実装がどこまで追従しているかを固定する。

## 2. 参照元

- `external/RVO2/src/Agent.cc`
- `src/cmis_ca/algorithms/orca/constraints.py`
- `src/cmis_ca/core/solver.py`

## 3. 追従済み項目

### 3.1 agent-agent ORCA line

- no-collision の cut-off circle 分岐
- no-collision の left leg / right leg 分岐
- collision 中の `timeStep` ベース分岐
- `line.point = velocity + 0.5 * u`

### 3.2 solver

- `linearProgram1/2/3` の式構造
- parallel 判定の `RVO_EPSILON = 1e-5`
- `linearProgram2` / `linearProgram3` の violation 判定を upstream 同様に厳密比較で扱う
- `protected_constraint_count` による obstacle line 優先の fallback

## 4. 回帰で固定した代表ケース

### 4.1 agent-agent

- 正面衝突前の non-collision leg 分岐
- collision 中の cut-off circle
- moving neighbor に対する cut-off circle 分岐
- left leg 分岐
- right leg 分岐

### 4.2 solver

- 単一制約への射影
- 複数制約の交点
- 反対向き parallel line の扱い
- `protected_constraint_count` により fallback 結果が変わるケース

## 5. 現時点で残す差分

- Python 実装はゼロ長ベクトルで NaN を出さないため、`_normalized_or()` に fallback を持つ
- `NaiveNeighborSearch` は kd-tree ではないため、neighbor 採用順の境界ケースは upstream と完全一致を保証しない
- solver の public API は中立名へ再構成しており、upstream の関数境界そのものではない

## 6. 結論

agent-agent constraint と solver の主要分岐は、現時点で upstream の式と分岐構造に概ね追従している。  
残る主要 gap は、neighbor semantics を含む実測比較の拡張と、複数 scenario での upstream 回帰強化である。
