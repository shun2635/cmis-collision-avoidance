# 由来コードと改変記録の方針

## 対象

この文書は、upstream 由来コードを本リポジトリへ取り込む場合の運用方針を定めます。

## 基本方針

1. `external/RVO2` は参照専用とする
2. upstream 由来コードを研究室実装側へコピーまたは改変して持ち込む場合は、元ライセンス条件を保持する
3. 改変したファイルには、研究室側で変更を加えた事実を明記する

## 記録する内容

最低限、以下をファイルヘッダまたは近接する文書に残します。

- upstream の出典
- 適用ライセンス
- このリポジトリで改変した事実
- 可能であれば改変日と改変概要

## 記録例

```text
Derived from: https://github.com/snape/RVO2
Original license: Apache License 2.0
Modified for the CMIS Collision Avoidance project.
Summary of changes: translated comments, renamed APIs, restructured logic.
```

## 新規作成ファイル

研究室で新規に作成したファイルは、このリポジトリ全体のライセンス方針に従います。  
詳細は [LICENSE](../../LICENSE) を参照してください。

## 文書更新との関係

由来コードの取り込みや改変を行った場合は、該当ファイルのヘッダだけでなく、必要に応じて仕様書やアルゴリズム文書も同じ change で更新する。  
運用の全体方針は [documentation-sync-policy.md](documentation-sync-policy.md) を参照してください。
