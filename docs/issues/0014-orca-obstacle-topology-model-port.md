# Issue 0014: ORCA の obstacle topology を upstream 寄りに再設計する

- ステータス: open
- 優先度: high
- 関連文書:
  - [docs/architecture/orca-reproduction-roadmap.md](../architecture/orca-reproduction-roadmap.md)
  - [docs/algorithms/orca.md](../algorithms/orca.md)
  - [external/RVO2/src/Obstacle.cc](../../external/RVO2/src/Obstacle.cc)
  - [external/RVO2/src/Agent.cc](../../external/RVO2/src/Agent.cc)

## 背景

現行実装の障害物は `ObstacleSegment` 単体であり、upstream の linked obstacle graph と凸性判定を持っていない。  
この差分が obstacle ORCA line の closest-point 近似につながっている。

## 目的

障害物表現を upstream に近い topology へ再設計し、正確な obstacle ORCA 制約生成の前提を作る。

## スコープ

- obstacle vertex / edge の表現を設計する
- 前後リンク、凸性、方向情報を保持できるモデルへ更新する
- scenario schema と loader を新構造へ対応させる
- 既存 `ObstacleSegment` 利用箇所を移行する
- docs と tests を更新する

## 非スコープ

- obstacle ORCA 制約ロジック本体の厳密移植
- 可視化

## 完了条件

- upstream obstacle model に必要な情報を Python 側で保持できる
- scenario file から障害物 polygon / chain を読める
- 旧 `ObstacleSegment` 近似に依存しない土台が整う

## 想定成果物

- `src/cmis_ca/core/world.py`
- `src/cmis_ca/io/scenario_loader.py`
- `scenarios/`
- `tests/core/`
- `docs/specifications/`

## 依存関係

- [0013-orca-agent-and-simulator-parity-audit.md](0013-orca-agent-and-simulator-parity-audit.md): pending
