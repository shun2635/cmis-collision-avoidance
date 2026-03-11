# Issue 0016: ORCA の agent constraint と solver の upstream 差分を詰める

- ステータス: open
- 優先度: high
- 関連文書:
  - [docs/architecture/orca-reproduction-roadmap.md](../architecture/orca-reproduction-roadmap.md)
  - [docs/algorithms/orca.md](../algorithms/orca.md)
  - [external/RVO2/src/Agent.cc](../../external/RVO2/src/Agent.cc)

## 背景

agent-agent 制約生成と共通 solver は既に移植済みだが、現時点では「基準実装」であり upstream 完全一致を保証していない。  
完全再現を目指すなら、collision / non-collision 分岐、fallback、neighbor 採用結果を含めて監査する必要がある。

## 目的

agent-agent ORCA line と solver 周辺の差分を洗い出して埋め、upstream に近い挙動へ収束させる。

## スコープ

- agent-agent constraint 生成の分岐監査
- `solve_linear_constraints()` の差分監査
- 必要なら near-parity の補助 API を追加する
- head-on 以外の対称ケースや collision 中ケースの回帰を増やす
- docs と tests を更新する

## 非スコープ

- obstacle topology の移植
- scenario schema 拡張

## 完了条件

- major branch の差分が docs に記録されている
- agent-agent / solver の回帰テストが拡充されている
- 現行 ORCA が upstream に対してどこまで一致しているか説明できる

## 想定成果物

- `src/cmis_ca/core/solver.py`
- `src/cmis_ca/algorithms/orca/constraints.py`
- `tests/algorithms/`
- `tests/core/`
- `docs/specifications/`

## 依存関係

- [0013-orca-agent-and-simulator-parity-audit.md](0013-orca-agent-and-simulator-parity-audit.md): pending
