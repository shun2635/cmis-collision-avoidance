# Issue 0015: ORCA の obstacle constraint 生成を upstream 準拠で移植する

- ステータス: open
- 優先度: high
- 関連文書:
  - [docs/architecture/orca-reproduction-roadmap.md](../architecture/orca-reproduction-roadmap.md)
  - [docs/algorithms/orca.md](../algorithms/orca.md)
  - [external/RVO2/src/Agent.cc](../../external/RVO2/src/Agent.cc)

## 背景

現行の obstacle ORCA line は current obstacle model に合わせた closest-point 近似であり、upstream 完全再現ではない。  
obstacle topology を upstream 寄りに持てるようにした後は、この近似を撤去する必要がある。

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

- [0014-orca-obstacle-topology-model-port.md](0014-orca-obstacle-topology-model-port.md): pending
