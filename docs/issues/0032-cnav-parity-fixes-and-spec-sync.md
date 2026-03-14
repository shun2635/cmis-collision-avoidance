# Issue 0032: CNav の parity 修正と docs 同期を行う

- ステータス: completed
- 優先度: medium
- 関連文書:
  - [docs/issues/0027-cnav-my-style-validation.md](0027-cnav-my-style-validation.md)
  - [docs/issues/0031-cnav-legacy-parity-gap-review.md](0031-cnav-legacy-parity-gap-review.md)
  - [docs/specifications/cnav-initial-implementation.md](../specifications/cnav-initial-implementation.md)

## 背景

gap review まで終わっても、採用した修正を実装と docs へ反映しなければ repo の状態は更新されない。  
この repo では、コードと docs を同じ change で同期する必要がある。

## 目的

gap review で採用した差分だけを Python 実装へ反映し、tests、仕様書、README を同期する。

## スコープ

- `src/cmis_ca/algorithms/cnav/` の必要な修正
- 追加 / 更新 test
- `docs/specifications/` の同期
- `README.md` の更新

## 非スコープ

- 採用しなかった legacy ヒューリスティクスの全面移植
- 未分類差分の調査継続

## 完了条件

- 採用した修正がコードに反映されている
- 関連 test が追加または更新されている
- 仕様書と README が実装内容に追従している
- issue `0031` の判断結果と矛盾しない

## 想定成果物

- `src/cmis_ca/algorithms/cnav/` の更新
- `tests/algorithms/` の更新
- `docs/specifications/` の更新
- `README.md` の更新

## 作業メモ

- 変更は gap review の採用項目に限定する
- 実装と docs の同期を崩さない

## 実施メモ

- `CNavParameters` に `update_every_step` を追加し、mainline default を変えずに毎 step 再評価を選べるようにした
- `create_cnav_parameters("legacy-forpaper-comparison")` を追加し、legacy 比較用 preset を実装した
- `scripts/dump_cnav_trace.py` に `--profile` を追加し、trace 比較時に named preset を選べるようにした
- `tests/algorithms/test_cnav.py` と `tests/algorithms/test_cnav_trace.py` に preset と every-step 更新の test を追加した
- `docs/specifications/` と `README.md` を新しい preset と trace 導線に合わせて同期した
