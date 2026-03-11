# Issue 0003: Poetry 環境へ切り替える

- ステータス: completed
- 優先度: medium
- 関連文書:
  - [README.md](../../README.md)
  - [pyproject.toml](../../pyproject.toml)
  - [docs/issues/0001-python-bootstrap.md](0001-python-bootstrap.md)

## 背景

Python 側の最小スケルトンは追加できたが、現状の `pyproject.toml` は Poetry 前提ではなく、開発環境の再現方法がまだ曖昧である。  
研究室内でメンバーが同じ手順で環境を作れるようにするには、依存管理と実行導線を Poetry に寄せておく方が扱いやすい。

## 目的

このリポジトリの Python 開発環境を Poetry 前提へ切り替え、今後の `install` `run` `test` の導線を Poetry ベースで統一する。

## スコープ

- `pyproject.toml` を Poetry 構成へ切り替える
- 必要なら `poetry.toml` を追加し、`.venv` をプロジェクト内へ作る方針を明示する
- README の実行例と開発手順を Poetry 前提へ更新する
- 現時点で可能な範囲の確認を行う

## 非スコープ

- 依存パッケージの追加
- ORCA 本実装
- lock file の厳密運用方針の確定

## 完了条件

- `pyproject.toml` が Poetry 前提の構成になっている
- Poetry で CLI とテストを実行する想定が README に反映されている
- `.venv` をどう扱うかが設定または文書で明示されている
- Poetry 未導入環境で未確認の項目があれば issue に記録されている

## 想定成果物

- `pyproject.toml`
- `poetry.toml` または README 上の運用記述
- 更新された README

## 作業メモ

- Poetry 本体が環境に無い場合は、lock file 生成は未実施として記録する
- Python バージョン制約は現在の実行環境に合わせて `>=3.9` を維持する
- `cmis-ca` CLI と `unittest` / `pytest` の導線は Poetry から呼べる形にする

## 実施結果

- `pyproject.toml` を `poetry-core` ベースへ切り替えた
- `poetry.toml` を追加し、`.venv` をプロジェクト内に作る方針を明示した
- README の導線を Poetry 前提へ更新した
- この環境には Poetry 本体が入っていなかったため、`poetry install` と lock file 生成は未実施
