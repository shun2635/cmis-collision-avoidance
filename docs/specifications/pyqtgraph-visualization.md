# PyQtGraph 可視化基盤

## 1. 目的

この文書は、issue `0021-fast-simulation-visualization` で追加した PyQtGraph ベースの可視化基盤を記録する。  
目的は、`Scenario` と `SimulationResult.history` から、障害物配置と agent 位置推移を高速に再生できるローカル viewer を提供することである。

## 2. 構成

| 種別 | パス | 内容 |
| --- | --- | --- |
| data model | `src/cmis_ca/visualization/models.py` | 可視化用 trace / frame / obstacle primitive |
| trace builder | `src/cmis_ca/visualization/trace_builder.py` | `Scenario` と `SimulationResult` を可視化用フレーム列へ変換 |
| viewer backend | `src/cmis_ca/visualization/pyqtgraph_viewer.py` | PyQtGraph による 2D 再生 viewer |
| CLI 導線 | `src/cmis_ca/cli/visualize.py` | scenario 実行から viewer 起動までを接続 |

## 3. 現在の入出力

### 3.1 入力

- `Scenario`
- `SimulationResult`

### 3.2 出力

- `VisualizationTrace`
  - `scenario_name`
  - `algorithm`
  - `time_step`
  - `agent_names`
  - `agent_radii`
  - `initial_positions`
  - `obstacles`
  - `frames`

## 4. viewer の現在仕様

- 障害物を polyline / polygon として描画する
- 初期位置を半透明の灰色 scatter として描画する
- 現在 frame の agent 位置を scatter として描画する
- 再生済み軌跡を polyline として描画する
- `play / pause`
- frame slider
- playback fps の変更
- auto fit された 2D view

## 5. CLI の現在仕様

追加したコマンドは以下。

```bash
poetry run cmis-ca visualize --algorithm orca --scenario scenarios/head_on.yaml
poetry run cmis-ca visualize --algorithm orca --scenario scenarios/upstream_circle.yaml --steps 8
```

サポート引数:

- `--algorithm`
- `--scenario`
- `--steps`
- `--fps`

## 6. 依存

現在の viewer backend は以下に依存する。

- `numpy`
- `pyqtgraph`
- `PySide6`

これらは lazy import されるため、非 GUI テストや数値実行時には import されない。

補足:

- `numpy`、`pyqtgraph`、`PySide6` は Poetry では Python 制約付き依存として扱う
- `numpy` は Python 3.13 / 3.14 で古い source build を避けるため、Python バージョンごとに互換系列を固定している
- 現行設定では GUI viewer 依存は `>=3.9,<3.15` の範囲で解決される
- Python `>=3.15` では本体の数値実行は維持しつつ、viewer は未対応とする

## 7. テスト方針

現在の自動確認は以下に限定する。

- `tests/visualization/test_trace_builder.py`
  - history から frame 変換できること
  - obstacle topology から obstacle primitive を復元できること
  - frame 件数不整合を弾くこと
- `tests/cli/test_main.py`
  - `visualize` subcommand が引数を受け取り、viewer 導線へ渡すこと

viewer 自体の表示確認は manual smoke とする。

## 8. manual smoke

1. `poetry install`
2. `poetry run cmis-ca visualize --algorithm orca --scenario scenarios/head_on.yaml`
3. agent 位置、障害物、slider、play / pause が動作することを確認する

## 9. 制約

- 現在はローカル viewer のみで、動画 export / HTML export は未実装
- agent 半径は trace に保持しているが、現 viewer では画面上の data-scale circle としては描いていない
- GUI backend が必要なため、headless CI での完全自動 UI テストは行っていない
