# Issue 0036: CNav legacy decision profile を切り出す

- ステータス: todo
- 優先度: medium
- 関連文書:
  - [0027-cnav-my-style-validation.md](0027-cnav-my-style-validation.md)
  - [0031-cnav-legacy-parity-gap-review.md](0031-cnav-legacy-parity-gap-review.md)
  - [0032-cnav-parity-fixes-and-spec-sync.md](0032-cnav-parity-fixes-and-spec-sync.md)
  - [../specifications/cnav-legacy-parity-gap-review.md](../specifications/cnav-legacy-parity-gap-review.md)

## 背景

現状の `legacy-forpaper-comparison` は cadence / horizon / coordination factor だけを寄せている。  
しかし legacy 差分の大きい部分は、まだ未対応の

- 17 action set
- mixed-speed action (`1.5`, `0.75` または `0.1`, `0.0`)
- legacy reward / politeness pipeline
- `maxNumSimulateNeighbors` / `maxNumThinkNeighbors`

である。

## 目的

mainline default を崩さずに、legacy 実験系の decision model を opt-in profile として切り出せるか判断し、必要なら実装する。

## スコープ

- 17 action set と speed table の設計
- legacy reward / politeness の責務分解
- simulate / think neighbor cap の導入方針整理
- named profile と test 方針の定義
- docs の同期

## 非スコープ

- mainline default の変更
- temporary goal / guidance の移植
- C++ trace exporter

## 完了条件

- legacy decision model を profile として切り出す方針が決まっている
- 採用する場合は opt-in 実装と tests が入っている
- 採用しない場合も、非採用理由が docs に記録されている

## 想定成果物

- `src/cmis_ca/algorithms/cnav/` の profile 拡張
- `tests/algorithms/` の更新
- `docs/specifications/` と `README.md` の更新

## 作業メモ

- `legacy-forpaper-comparison` を拡張するか、新しい profile 名に分けるかを先に決める
- reward pipeline は一括移植ではなく、比較効果の大きい要素から段階導入する
- mainline `paper` profile との境界を docs で明示する
