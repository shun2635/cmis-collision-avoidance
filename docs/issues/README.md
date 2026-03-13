# Issue 運用

このディレクトリでは、研究室内で次に取り組む作業を issue 文書として管理します。

## 基本方針

- issue は `docs/issues/NNNN-short-title.md` 形式で追加する
- `NNNN` は 4 桁の連番とする
- 1 issue につき 1 つの達成対象を定める
- 設計変更や実装方針の変更は、実装前に issue と関連文書へ反映する
- 実装を行った issue は、完了前に関連する仕様書と docs の更新を必須とする

## 推奨項目

- タイトル
- ステータス
- 優先度
- 背景
- 目的
- スコープ
- 非スコープ
- 完了条件
- 想定成果物
- 作業メモ

## 現在の issue

- [0001-python-bootstrap.md](0001-python-bootstrap.md)
- [0002-orca-boundary-definition.md](0002-orca-boundary-definition.md)
- [0003-poetry-environment-migration.md](0003-poetry-environment-migration.md)
- [0004-core-reference-types-and-tests.md](0004-core-reference-types-and-tests.md)
- [0005-core-neighbor-search-reference-implementation.md](0005-core-neighbor-search-reference-implementation.md)
- [0006-core-linear-solver-port.md](0006-core-linear-solver-port.md)
- [0007-orca-constraint-generation.md](0007-orca-constraint-generation.md)
- [0008-orca-step-integration.md](0008-orca-step-integration.md)
- [0009-scenario-schema-and-loader.md](0009-scenario-schema-and-loader.md)
- [0010-upstream-circle-scenario-regression.md](0010-upstream-circle-scenario-regression.md)
- [0011-pytest-test-suite-migration.md](0011-pytest-test-suite-migration.md)
- [0012-orca-goal-model-and-pref-velocity-alignment.md](0012-orca-goal-model-and-pref-velocity-alignment.md)
- [0013-orca-agent-and-simulator-parity-audit.md](0013-orca-agent-and-simulator-parity-audit.md)
- [0014-orca-obstacle-topology-model-port.md](0014-orca-obstacle-topology-model-port.md)
- [0015-orca-obstacle-constraint-exact-port.md](0015-orca-obstacle-constraint-exact-port.md)
- [0016-orca-agent-constraint-and-solver-parity.md](0016-orca-agent-constraint-and-solver-parity.md)
- [0017-orca-upstream-regression-suite-expansion.md](0017-orca-upstream-regression-suite-expansion.md)
- [0018-orca-parity-gap-review.md](0018-orca-parity-gap-review.md)
- [0019-orca-neighbor-search-parity-audit.md](0019-orca-neighbor-search-parity-audit.md)
- [0020-orca-roadmap-regression-support.md](0020-orca-roadmap-regression-support.md)
- [0021-fast-simulation-visualization.md](0021-fast-simulation-visualization.md)
- [0022-cnav-initial-implementation.md](0022-cnav-initial-implementation.md)
- [0023-cnav-orca-per-agent-solve-helper.md](0023-cnav-orca-per-agent-solve-helper.md)
- [0024-cnav-action-model-and-intent-cache.md](0024-cnav-action-model-and-intent-cache.md)
- [0025-cnav-coordination-evaluation.md](0025-cnav-coordination-evaluation.md)
- [0026-cnav-cli-scenarios-tests-and-spec.md](0026-cnav-cli-scenarios-tests-and-spec.md)

## 現在の優先順

ORCA 完全再現を優先する。順序は [../architecture/orca-reproduction-roadmap.md](../architecture/orca-reproduction-roadmap.md) を正とする。

1. [0012-orca-goal-model-and-pref-velocity-alignment.md](0012-orca-goal-model-and-pref-velocity-alignment.md) `completed`
2. [0013-orca-agent-and-simulator-parity-audit.md](0013-orca-agent-and-simulator-parity-audit.md) `completed`
3. [0014-orca-obstacle-topology-model-port.md](0014-orca-obstacle-topology-model-port.md) `completed`
4. [0015-orca-obstacle-constraint-exact-port.md](0015-orca-obstacle-constraint-exact-port.md) `completed`
5. [0016-orca-agent-constraint-and-solver-parity.md](0016-orca-agent-constraint-and-solver-parity.md) `completed`
6. [0017-orca-upstream-regression-suite-expansion.md](0017-orca-upstream-regression-suite-expansion.md) `completed`
7. [0018-orca-parity-gap-review.md](0018-orca-parity-gap-review.md) `completed`
8. [0019-orca-neighbor-search-parity-audit.md](0019-orca-neighbor-search-parity-audit.md) `completed`
9. [0020-orca-roadmap-regression-support.md](0020-orca-roadmap-regression-support.md) `completed`

## ORCA 優先期間後の候補

- `proxemic`
- 多アルゴリズム比較の広い API 拡張
- `cnav` の初期導入は [0022-cnav-initial-implementation.md](0022-cnav-initial-implementation.md) とその子 issue で完了した
