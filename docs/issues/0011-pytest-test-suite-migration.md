# Issue 0011: テスト実行を pytest 基準へ移行する

- ステータス: completed
- 優先度: medium
- 関連文書:
  - [README.md](../../README.md)
  - [pyproject.toml](../../pyproject.toml)
  - [docs/specifications/python-skeleton-detailed-design.md](../specifications/python-skeleton-detailed-design.md)
  - [tests/core/test_geometry.py](../../tests/core/test_geometry.py)
  - [tests/core/test_state.py](../../tests/core/test_state.py)
  - [tests/core/test_world.py](../../tests/core/test_world.py)
  - [tests/core/test_neighbor_search.py](../../tests/core/test_neighbor_search.py)
  - [tests/test_simulator_smoke.py](../../tests/test_simulator_smoke.py)

## 背景

現状のテストは `unittest` ベースで動いているが、このリポジトリでは Python 実装を主対象としており、今後は fixture や parameterize を使うテストが増える見込みである。  
また、研究室内での可読性と記述量を考えると、`assert` ベースで書ける `pytest` の方が保守しやすい。

## 目的

テスト実行の標準を `pytest` に切り替え、新規テストを `pytest` スタイルで追加できる状態にする。  
同時に、README と仕様書の実行手順も `pytest` 前提へ更新する。

## スコープ

- 既存 Python テストを `pytest` スタイルへ寄せる
- README のテスト実行コマンドを `pytest` 前提へ更新する
- 仕様書のテスト方針と実行手順を更新する
- `docs/issues/README.md` に issue を追加する

## 非スコープ

- テスト対象の大幅追加
- property-based testing の導入
- coverage しきい値の設定
- C++ 側テストの再編

## 完了条件

- `tests/` 配下の Python テストが `pytest` で通る
- 主要テストが `unittest.TestCase` に依存しない
- README のテスト実行例が `pytest` になっている
- 仕様書が現在のテスト実行方法を反映している

## 想定成果物

- `docs/issues/0011-pytest-test-suite-migration.md`
- `tests/core/*.py`
- `tests/test_simulator_smoke.py`
- 更新された README / docs

## 作業メモ

- `pytest` は既に依存関係へ入っているため、今回はテストコードと docs の整合が主作業になる
- fixture は無理に増やさず、まずは読みやすい関数テストへ寄せる
- 既存の振る舞いを変えずに runner と記述様式を揃える
