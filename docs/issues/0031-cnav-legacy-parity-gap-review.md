# Issue 0031: CNav の legacy parity gap をレビューする

- ステータス: todo
- 優先度: medium
- 関連文書:
  - [docs/issues/0027-cnav-my-style-validation.md](0027-cnav-my-style-validation.md)
  - [docs/issues/0030-cnav-trace-parity-harness.md](0030-cnav-trace-parity-harness.md)
  - [docs/algorithms/cnav.md](../algorithms/cnav.md)

## 背景

trace と scenario が揃っても、その差分を「直すべきバグ」と「legacy 固有の癖」に分けないと実装判断がぶれる。  
ORCA 側でも parity gap review を先に作ったことで、後続の修正判断が安定した。

## 目的

CNav の parity gap を整理し、採用判断と修正優先度を文書化する。

## スコープ

- trace 比較結果の整理
- 差分の分類
- 修正優先度の設定
- docs へ残す intentional difference の抽出

## 非スコープ

- 実際のコード修正
- scenario 追加

## 完了条件

- 主要差分が列挙されている
- 各差分に `fix` / `intentional difference` / `legacy-only heuristic` の分類が付いている
- 後続修正の優先度が決まっている
- 関連 docs から参照できる

## 想定成果物

- parity gap review 文書
- 修正優先度リスト

## 作業メモ

- legacy に合わせること自体を目的にしない
- 論文、legacy、現行 Python の 3 者を分けて書く
