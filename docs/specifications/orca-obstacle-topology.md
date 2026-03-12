# ORCA obstacle topology モデル

## 1. 目的

この文書は、現行 Python 実装で採用している obstacle topology を固定する。  
対象は obstacle ORCA line 本体ではなく、その前提となる linked vertex model と scenario schema である。

## 2. 現在の runtime モデル

runtime では、障害物は `ObstacleVertex` の flat tuple として保持する。

各 vertex は以下を持つ。

- `point`
- `obstacle_id`
- `vertex_id`
- `previous_index`
- `next_index`
- `direction`
- `is_convex`

これは upstream `Obstacle` の `point_`, `previous_`, `next_`, `direction_`, `isConvex_` に対応する最小表現である。

## 3. scenario 入力モデル

scenario file では、障害物を `ObstaclePath` 相当として記述する。

```yaml
obstacles:
  - closed: false
    vertices:
      - [0.6, -1.0]
      - [0.6, 1.0]
```

### 現在のルール

- `vertices` は 2 点以上必須
- `closed` は省略時 `true`
- `closed: true` のとき、始点の重複再掲は許可しない
- consecutive duplicate vertex は禁止

## 4. topology 構築規則

`build_obstacle_topology()` は `ObstaclePath` 群から flat vertex tuple を作る。

- closed polygon:
  - `previous_index` と `next_index` は循環する
  - `is_convex` は隣接 3 点の向きから計算する
- open chain:
  - 先頭は `previous_index = None`
  - 末尾は `next_index = None`
  - `is_convex = True` とする

## 5. 現在の利用箇所

- `core/neighbor_search.py`
  - `next_index` を持つ vertex のみを obstacle edge として距離計算する
- `algorithms/orca/constraints.py`
  - linked vertex から edge を取得し、upstream obstacle line 分岐を Python へ移植して使う
- `io/scenario_loader.py`
  - `vertices + closed` から topology を構築する

## 6. 未対応事項

- obstacle kd-tree は未対応
- loader は互換のため `start` / `end` 2 点定義も受け付けるが、正規 schema は `vertices` 形式とする
- open chain は研究室都合の拡張であり、endpoint 周りの意味は upstream の閉 polygon と完全には一致しない

## 7. 次段階

obstacle topology の次段階は、[../issues/0017-orca-upstream-regression-suite-expansion.md](../issues/0017-orca-upstream-regression-suite-expansion.md) に進み、この topology と obstacle line 実装が複数 scenario でどう振る舞うかを回帰 suite で確認することになる。
