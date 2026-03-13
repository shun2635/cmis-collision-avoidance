# ORCA 完全再現ロードマップ

## 目的

この文書は、当面の最優先事項を `ORCA の upstream 再現度向上` に置くことを明文化する。  
`proxemic` と `cnav` は構造上の拡張先として残すが、着手は ORCA の再現が一段落した後とする。

## 現時点の判断

- まず ORCA を upstream `RVO2` にできるだけ近い形で再現する
- 研究室向けの読みやすさと docs 同期の方針は維持する
- 多アルゴリズム対応の構造は残すが、追加アルゴリズムは ORCA を下層基準として積む
- `proxemic` `cnav` の詳細設計は、ORCA 優先 issue 完了後に進める

## 既に到達した段階

- Python ベースの実装基盤
- 共通 core
- ORCA の最小 1 step 統合
- シナリオローダ
- obstacle topology の linked vertex 化
- obstacle ORCA constraint の upstream ベース移植
- upstream `Circle`、`Blocks`、`Roadmap` に基づく回帰基盤

詳細は [../specifications/python-skeleton-detailed-design.md](../specifications/python-skeleton-detailed-design.md)、[../specifications/upstream-circle-regression.md](../specifications/upstream-circle-regression.md)、[../specifications/upstream-blocks-regression.md](../specifications/upstream-blocks-regression.md)、[../specifications/upstream-roadmap-regression.md](../specifications/upstream-roadmap-regression.md) を参照する。

## 残る主要ギャップ

### 1. regression の厳密度

現在の upstream 比較は `Circle`、`Blocks`、`Roadmap` まで増えたが、まだ定性的な確認が中心である。  
今後は step-by-step 比較や長時間実行の評価など、より厳密な回帰条件を整備する余地がある。

### 2. obstacle 周辺の残課題

obstacle line 本体は移植済みだが、obstacle kd-tree や open chain の取り扱いは upstream そのものではない。  
完全再現の最終段階では、これらの差分が実測に与える影響も見直す必要がある。

### 3. neighbor data structure の差分

neighbor semantics 自体は upstream ベースで揃えたが、実装は依然として `NaiveNeighborSearch` である。  
性能特性と、kd-tree 探索順に依存する完全同率ケースの順序は、なお受容差分として残る。

## ORCA 優先 issue 順

1. `0012-orca-goal-model-and-pref-velocity-alignment` `completed`
2. `0013-orca-agent-and-simulator-parity-audit` `completed`
3. `0014-orca-obstacle-topology-model-port` `completed`
4. `0015-orca-obstacle-constraint-exact-port` `completed`
5. `0016-orca-agent-constraint-and-solver-parity` `completed`
6. `0017-orca-upstream-regression-suite-expansion` `completed`
7. `0018-orca-parity-gap-review` `completed`
8. `0019-orca-neighbor-search-parity-audit` `completed`
9. `0020-orca-roadmap-regression-support` `completed`

## 後回しにするもの

- `proxemic`
- `cnav`
- アルゴリズム比較 UI / 可視化の拡張
- ORCA 完全再現と直接関係しない API 拡張
