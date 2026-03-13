# Issue 0026: CNav の CLI・scenario・tests・仕様書を仕上げる

- ステータス: todo
- 優先度: medium
- 関連文書:
  - [docs/algorithms/cnav.md](../algorithms/cnav.md)
  - [docs/issues/0022-cnav-initial-implementation.md](0022-cnav-initial-implementation.md)
  - [README.md](../../README.md)

## 背景

CNav 本体が動いても、registry、CLI、scenario、tests、仕様書が揃わなければ repo の正式機能として扱えない。  
このリポジトリでは、実装と docs の同期を同一 change で完了させる必要がある。

## 目的

CNav を repo の利用可能アルゴリズムとして露出し、最小 scenario、tests、仕様書まで含めて導入を完了させる。

## スコープ

- `AlgorithmRegistry` への `cnav` 登録
- CLI からの `--algorithm cnav` 実行導線
- 最小 scenario の追加
- unit test / smoke test の追加
- `docs/specifications/` の CNav 初期仕様書追加
- README 更新

## 非スコープ

- 大規模回帰ベンチマーク
- すべての論文 scenario の再現

## 完了条件

- `create_algorithm(\"cnav\")` が動く
- `cmis-ca run --algorithm cnav --scenario ...` が動く
- CNav 向け最小 scenario が追加されている
- CNav 固有 test が追加されている
- 実装仕様が docs に固定されている

## 想定成果物

- `src/cmis_ca/algorithms/registry.py`
- `tests/algorithms/` の CNav test
- `scenarios/` の CNav scenario
- `docs/specifications/` の CNav 仕様書
- `README.md`

## 作業メモ

- 初回は比較 metrics ではなく smoke と単体 test を優先する
- README では「実験的導入」か「利用可能」かの表現を実装状態に合わせる
