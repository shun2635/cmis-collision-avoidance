# ORCA parity gap review

## 1. 目的

この文書は、issue `0012` から `0020` までで到達した ORCA 再現度を棚卸しし、未解決の差分と次段階の判断基準を固定する。

## 2. 現時点の到達点

2026-03-12 時点で、以下は実装済みである。

- goal / preferred velocity モデル
- agent / simulator の主要 parameter と clock
- obstacle topology の linked vertex 化
- obstacle ORCA line の upstream ベース移植
- agent-agent ORCA line の主要分岐
- solver の主要分岐
- upstream `Circle` / `Blocks` / `Roadmap` の定性的 regression

## 3. 残差一覧

| 項目 | 現在の状態 | 影響 | 判定 |
| --- | --- | --- | --- |
| neighbor search data structure | semantics は upstream ベースで揃えたが kd-tree ではない | 性能特性と完全同率ケースの探索順が upstream と一致しない可能性がある | accepted |
| open chain semantics | 研究室拡張として一般化 | upstream の閉 polygon 前提と完全には一致しない | medium |
| regression strength | `Circle` と `Blocks` と `Roadmap` の定性的比較まで | 厳密な parity 判定にはまだ弱い | high |
| roadmap guidance implementation | regression helper 内の local visibility graph で扱う | public API や core planner とはまだ分離されている | accepted |
| example-specific perturbation | `Circle.cc` / `Blocks.cc` の微小 perturbation は未導入 | 対称ケースの deadlock 回避や tie-break が完全一致しない | low |
| public API shape | setter 群や default agent object は Python らしく再構成 | API 形は違うが ORCA の挙動再現そのものには直結しない | accepted |

## 4. docs 整合性の結論

- ORCA の主要幾何ロジックは `orca-agent-solver-parity.md` と `orca-obstacle-topology.md` に反映済み
- neighbor semantics は `orca-neighbor-search-parity.md` に反映済み
- regression は `upstream-circle-regression.md`、`upstream-blocks-regression.md`、`upstream-roadmap-regression.md` に分離され、入口が明確になった
- 旧状態の「obstacle line 未移植」「neighbor semantics 未監査」「Circle のみ」「Roadmap 未対応」という記述は更新済み

## 5. 進行判断

### 5.1 現時点で言えること

- `研究室向け ORCA 基盤` としては十分に成立している
- `upstream 完全再現が終わった` と断言できる段階ではない

### 5.2 multi-algorithm へ進む判断

現時点では、`proxemic` / `cnav` を mainline の次優先へ上げるのはまだ早い。  
理由は、regression strength に未解決の差分が残っており、共通コアの正しさを ORCA で十分に詰め切っていないためである。

## 6. multi-algorithm 着手の条件

以下を満たした時点で、ORCA 優先フェーズを一段落とみなす。

1. obstacle を含む scenario で回帰条件がもう 1 段強化されている
2. ORCA の残差が「受容する差分」と「未解決バグ候補」に分離されている
3. roadmap 系 scenario の比較条件が docs と tests で再現できる
4. local visibility helper を core API に上げるか、regression 専用で残すかを判断できる

## 7. follow-up issue

現時点では、ORCA 優先フェーズの必須 issue は `0020` まで完了している。  
追加の follow-up は、accepted gap を core API へ昇格させるかどうかの判断が必要になった時点で切る。

## 8. 結論

ORCA は、研究室内で説明・実験・保守できる水準まで整っている。  
ただし upstream parity の最終確認としては、roadmap を含む regression 拡張を終えた現時点でも、なお accepted / medium の差分が残る。  
したがって `proxemic` / `cnav` へ主軸を移す場合も、この残差を明示したうえで進めるべきである。
