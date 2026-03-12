# ORCA 完全再現ロードマップ

## 目的

この文書は、当面の最優先事項を `ORCA の upstream 再現度向上` に置くことを明文化する。  
`proxemic` と `cnav` は構造上の拡張先として残すが、着手は ORCA の再現が一段落した後とする。

## 現時点の判断

- まず ORCA を upstream `RVO2` にできるだけ近い形で再現する
- 研究室向けの読みやすさと docs 同期の方針は維持する
- 多アルゴリズム対応の構造は残すが、直近 issue は ORCA に集中させる
- `proxemic` `cnav` の issue は新規には切らず、ORCA 完全再現後に再評価する

## 既に到達した段階

- Python ベースの実装基盤
- 共通 core
- ORCA の最小 1 step 統合
- シナリオローダ
- upstream `Circle` に基づく初期回帰基盤

詳細は [../specifications/python-skeleton-detailed-design.md](../specifications/python-skeleton-detailed-design.md) と [../specifications/upstream-circle-regression.md](../specifications/upstream-circle-regression.md) を参照する。

## 残る主要ギャップ

### 1. obstacle constraint の厳密移植

障害物 topology 自体は `ObstacleVertex` の linked model へ移行済みで、前後リンク、方向、凸性を保持できる。  
一方で obstacle ORCA line 本体はまだ closest-point 近似に留まっており、upstream の分岐は未移植である。

### 2. agent / solver / neighbor semantics の差分

agent / simulator の主要 parameter と clock は監査済みだが、agent-agent 制約生成と solver の完全一致監査はまだ終わっていない。  
近傍採用条件、分岐の細部、境界ケースを詰める必要がある。

### 3. 回帰 suite の薄さ

現在の upstream 比較は `Circle` に限定され、しかも定性的な確認が中心である。  
今後は scenario を増やし、より厳密な回帰条件を整備する必要がある。

## ORCA 優先 issue 順

1. `0012-orca-goal-model-and-pref-velocity-alignment` `completed`
2. `0013-orca-agent-and-simulator-parity-audit` `completed`
3. `0014-orca-obstacle-topology-model-port` `completed`
4. `0015-orca-obstacle-constraint-exact-port`
5. `0016-orca-agent-constraint-and-solver-parity`
6. `0017-orca-upstream-regression-suite-expansion`
7. `0018-orca-parity-gap-review`

## 後回しにするもの

- `proxemic`
- `cnav`
- アルゴリズム比較 UI / 可視化の拡張
- ORCA 完全再現と直接関係しない API 拡張
