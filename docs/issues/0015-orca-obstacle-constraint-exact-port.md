# Issue 0015: ORCA の obstacle constraint 生成を upstream 準拠で移植する

- ステータス: completed
- 優先度: high
- 関連文書:
  - [docs/architecture/orca-reproduction-roadmap.md](../architecture/orca-reproduction-roadmap.md)
  - [docs/algorithms/orca.md](../algorithms/orca.md)
  - [external/RVO2/src/Agent.cc](../../external/RVO2/src/Agent.cc)

## 背景

issue 着手時点では、obstacle topology は linked vertex 化されていたが、obstacle ORCA line 本体は closest-point 近似のままだった。  
upstream 完全再現を進めるには、`Agent.cc` の obstacle 分岐を直接移植する必要があった。

## 目的

obstacle ORCA constraint 生成を upstream の分岐に沿って移植し、vertex / edge / convexity を反映した正規実装へ置き換える。

## スコープ

- upstream obstacle line 生成の分岐を読み解く
- Python 側へ移植する
- 既存 closest-point 近似を削除または退役させる
- obstacle ケースのテストを詳細化する
- docs を更新する

## 非スコープ

- agent-agent line の監査
- regression suite 全体の拡張

## 完了条件

- `algorithms/orca/constraints.py` の obstacle 側が upstream 準拠へ置き換わる
- obstacle 関連テストが upstream の主要分岐をカバーする
- 仕様書に近似撤去の事実が反映される

## 想定成果物

- `src/cmis_ca/algorithms/orca/constraints.py`
- `tests/algorithms/`
- `docs/specifications/`

## 依存関係

- [0014-orca-obstacle-topology-model-port.md](0014-orca-obstacle-topology-model-port.md): completed

## 実施メモ

- `algorithms/orca/constraints.py` の obstacle 側を upstream `Agent.cc` の分岐に沿って移植した
- `alreadyCovered` 判定、vertex collision、segment collision、leg projection、foreign leg の扱いを反映した
- open chain endpoint は `previous_index` / `next_index` の有無で分岐し、研究室拡張として保守的に扱う
- obstacle 系の unit test を polygon ベースへ更新し、collision / non-collision / foreign leg を固定した
- `scenarios/obstacle_demo.yaml` も polygon 例へ更新した
