# Issue 0009: シナリオスキーマとローダを定義する

- ステータス: open
- 優先度: medium
- 関連文書:
  - [docs/repository-architecture.md](../repository-architecture.md)
  - [docs/api-draft.md](../api-draft.md)
  - [README.md](../../README.md)
  - [src/cmis_ca/core/world.py](../../src/cmis_ca/core/world.py)
  - [src/cmis_ca/cli/main.py](../../src/cmis_ca/cli/main.py)

## 背景

現状の CLI は built-in の最小デモだけを動かしており、`--scenario` は未実装である。  
研究室内でシナリオを共有し、同じ条件で複数アルゴリズムを比較するには、シナリオの記述形式とローダが必要になる。

## 目的

`scenarios/` で扱う最小スキーマを定義し、CLI から読み込めるローダを追加する。

## スコープ

- シナリオファイルの最小スキーマを決める
- 読み込みローダを追加する
- `--scenario` を CLI で有効化する
- サンプルシナリオを 1 つ以上追加する
- シナリオ読み込みテストを追加する

## 非スコープ

- 可視化
- 複雑な設定マージ機構
- upstream 比較

## 完了条件

- `scenarios/` にサンプルファイルがある
- CLI から `--scenario` を渡して実行できる
- スキーマ違反時の基本的なエラー処理がある
- 同一シナリオを将来の複数アルゴリズムで共有できる形になっている

## 想定成果物

- `scenarios/`
- `src/cmis_ca/io/` または同等のローダ実装
- `tests/integration/` または `tests/core/`

## 作業メモ

- 最初は JSON でも YAML でもよいが、研究室で読みやすい形式を優先する
- アルゴリズム固有設定はシナリオ本体と分離する
- built-in デモから外部ファイルへ移る最初の踏み台と考える

## 依存関係

- [0008-orca-step-integration.md](0008-orca-step-integration.md): pending
