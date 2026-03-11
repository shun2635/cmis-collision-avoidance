# Issue 0002: ORCA の共通コア境界を先に固定する

- ステータス: completed
- 優先度: high
- 関連文書:
  - [docs/architecture/repository-structure.md](../architecture/repository-structure.md)
  - [docs/architecture/overview.md](../architecture/overview.md)
  - [docs/architecture/api.md](../architecture/api.md)
  - [docs/algorithms/README.md](../algorithms/README.md)
  - [docs/issues/0001-python-bootstrap.md](0001-python-bootstrap.md)

## 背景

このリポジトリは、ORCA だけを再実装する場ではなく、proxemic や CNav などの関連アルゴリズムも同一基盤上で扱うことを前提としている。  
そのため、実装に入る前に「どこまでを共通コアとするか」「どこからを ORCA 固有差分とするか」を先に固定しないと、後続の Python 実装や issue 分解がぶれる。

特に ORCA は最初に着手する基準アルゴリズムであり、ここで責務分担を曖昧にしたまま進めると、後から proxemic や CNav を追加する際に再分解が必要になる。

## 目的

`docs/algorithms/orca.md` を作成し、ORCA の理論要素、実装要素、共通化可能な要素、ORCA 固有に閉じ込める要素を明文化する。  
この文書をもとに、`src/cmis_ca/core/` と `src/cmis_ca/algorithms/orca/` の責務境界を固定する。

## スコープ

- `docs/algorithms/orca.md` を新規作成する
- ORCA の計算フローを研究室向けに整理する
- ORCA の各構成要素について、`core/` に置くか `algorithms/orca/` に置くかを決める
- 将来の proxemic / CNav 追加を見据えた共通化判断基準を ORCA 観点で明文化する
- 必要に応じて `docs/architecture/repository-structure.md` と `docs/architecture/api.md` の整合を取る

## 非スコープ

- ORCA の Python 実装そのもの
- proxemic / CNav の詳細設計
- 数式導出の完全な教科書化
- upstream との挙動比較

## 完了条件

- `docs/algorithms/orca.md` が存在する
- ORCA の 1 ステップ処理が、研究室内で読める粒度で説明されている
- ORCA の各要素について、共通コアか ORCA 固有かの分類表がある
- `core/` と `algorithms/orca/` の責務境界が文書上で説明されている
- 後続の Python 実装 issue が、この文書を参照して着手できる状態になっている

## 想定成果物

- `docs/algorithms/orca.md`
- 必要に応じた `docs/architecture/repository-structure.md` の追記
- 必要に応じた `docs/architecture/api.md` の追記

## 作業メモ

- ORCA の説明は upstream のコード構造そのものではなく、研究室向けの責務単位で再整理する
- 少なくとも以下は分類対象に含める
  - ベクトル演算
  - 近傍探索
  - エージェント状態
  - 予測速度
  - ORCA line / 制約生成
  - 線形計画
  - シミュレーション更新
- 「ORCA で使っているが、他アルゴリズムでも再利用しそうなもの」は原則として `core/` 候補として扱う
- 「ORCA 特有の意味付けを持つもの」は `algorithms/orca/` に閉じ込める

## `orca.md` に含めたい項目

- ORCA の位置付け
- upstream との関係
- ORCA の 1 ステップ処理
- データ構造案
- 共通コアに入れるもの
- ORCA 固有差分として持つもの
- `src/cmis_ca/` へのマッピング案
- 将来の proxemic / CNav 追加時に衝突しそうな論点

## 後続との関係

この issue は、保留中の [0001-python-bootstrap.md](0001-python-bootstrap.md) より先に進める。  
`0001` を再開する前提条件のひとつとして、この issue の完了を置く。
