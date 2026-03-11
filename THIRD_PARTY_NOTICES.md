# THIRD_PARTY_NOTICES

この文書は、本リポジトリに含まれる third-party code と、その取り扱い方針をまとめたものです。

## 1. snape/RVO2

- 名称: RVO2 Library
- upstream URL: <https://github.com/snape/RVO2>
- 配置先: `external/RVO2`
- ライセンス: Apache License 2.0
- 用途: ORCA の挙動、API、アルゴリズム構成を確認するための参照用コード

### 補足

- `external/RVO2` は upstream を参照するための外部コードです。
- 研究室独自実装は `src/` 以下で進めます。
- 将来的に upstream 由来のコード片を研究室側へ移植または改変する場合は、元の著作権表示とライセンス条件を保持したうえで、改変内容を明示します。
- upstream 由来コードに対しては、Apache License 2.0 の条件が適用されます。

## 2. ライセンスの参照先

- リポジトリ全体のライセンス: [LICENSE](LICENSE)
- upstream のライセンス本文: [external/RVO2/LICENSE](external/RVO2/LICENSE)

## 3. 改変記録の方針

upstream 由来のファイルをコピーまたは改変して本リポジトリ内へ配置する場合は、少なくとも以下を記録します。

- 元ファイルの出典
- 適用されるライセンス
- 研究室側で加えた改変の事実
- 必要であれば改変日と改変概要

運用の詳細は [docs/policies/source-file-policy.md](docs/policies/source-file-policy.md) を参照してください。
