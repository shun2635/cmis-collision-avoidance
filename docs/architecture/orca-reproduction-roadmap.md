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
- obstacle topology の linked vertex 化
- obstacle ORCA constraint の upstream ベース移植
- upstream `Circle` と `Blocks` に基づく初期回帰基盤

詳細は [../specifications/python-skeleton-detailed-design.md](../specifications/python-skeleton-detailed-design.md)、[../specifications/upstream-circle-regression.md](../specifications/upstream-circle-regression.md)、[../specifications/upstream-blocks-regression.md](../specifications/upstream-blocks-regression.md) を参照する。

## 残る主要ギャップ

### 1. neighbor semantics の差分

agent-agent 制約生成と solver の主要分岐は監査済みだが、neighbor 採用条件は `NaiveNeighborSearch` のままである。  
探索順や打ち切り境界が upstream kd-tree とどこまで一致するかは、なお詰める必要がある。

### 2. 回帰 suite の薄さ

現在の upstream 比較は `Circle` と `Blocks` まで増えたが、まだ定性的な確認が中心である。  
今後は scenario をさらに増やし、より厳密な回帰条件を整備する必要がある。

### 3. obstacle 周辺の残課題

obstacle line 本体は移植済みだが、obstacle kd-tree や open chain の取り扱いは upstream そのものではない。  
完全再現の最終段階では、これらの差分が実測に与える影響も見直す必要がある。

## ORCA 優先 issue 順

1. `0012-orca-goal-model-and-pref-velocity-alignment` `completed`
2. `0013-orca-agent-and-simulator-parity-audit` `completed`
3. `0014-orca-obstacle-topology-model-port` `completed`
4. `0015-orca-obstacle-constraint-exact-port` `completed`
5. `0016-orca-agent-constraint-and-solver-parity` `completed`
6. `0017-orca-upstream-regression-suite-expansion` `completed`
7. `0018-orca-parity-gap-review`

## 後回しにするもの

- `proxemic`
- `cnav`
- アルゴリズム比較 UI / 可視化の拡張
- ORCA 完全再現と直接関係しない API 拡張
