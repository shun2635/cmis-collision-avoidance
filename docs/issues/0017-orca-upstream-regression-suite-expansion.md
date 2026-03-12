# Issue 0017: upstream ORCA scenario 回帰 suite を拡張する

- ステータス: open
- 優先度: medium
- 関連文書:
  - [docs/architecture/orca-reproduction-roadmap.md](../architecture/orca-reproduction-roadmap.md)
  - [docs/specifications/upstream-circle-regression.md](../specifications/upstream-circle-regression.md)
  - [external/RVO2/examples](../../external/RVO2/examples)

## 背景

現在の upstream 比較基盤は `Circle` の初期版に留まる。  
ORCA 完全再現を判断するには、複数 scenario を使った回帰 suite が必要になる。

## 目的

`Circle` に加えて upstream 由来 scenario を増やし、定性的比較からより強い回帰 suite へ拡張する。

## スコープ

- 対象 scenario を選定する
- Python 側へ比較用 scenario を追加する
- metric と pass condition を増やす
- `tests/regression/` または比較 script を拡充する
- docs を更新する

## 非スコープ

- 完全 benchmark 基盤
- proxemic / cnav の比較

## 完了条件

- `Circle` 以外の upstream 由来 scenario が 1 つ以上追加されている
- regression suite の観点が整理されている
- ORCA 差分確認の入口が docs で分かる

## 想定成果物

- `scenarios/`
- `tests/regression/`
- `scripts/compare_*`
- `docs/specifications/`

## 依存関係

- [0015-orca-obstacle-constraint-exact-port.md](0015-orca-obstacle-constraint-exact-port.md): completed
- [0016-orca-agent-constraint-and-solver-parity.md](0016-orca-agent-constraint-and-solver-parity.md): completed
