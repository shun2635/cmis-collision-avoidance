# Issue 0021: シミュレーション結果の高速可視化基盤を追加する

- ステータス: completed
- 優先度: medium
- 関連文書:
  - [docs/architecture/visualization-design.md](../architecture/visualization-design.md)
  - [docs/specifications/python-skeleton-detailed-design.md](../specifications/python-skeleton-detailed-design.md)
  - [README.md](../../README.md)

## 背景

現状の CLI と regression script は数値出力と test に寄っており、初期位置から各 time step の位置推移と障害物配置を視覚的に確認しにくい。  
研究室内での説明、デバッグ、比較実験には、エージェント軌跡と障害物をアニメーション的に確認できる導線が必要である。

一方で、`matplotlib` は生成も描画も遅く、エージェント数や step 数が増えると反復検証に向かない。  
そのため、より高速に生成・描画できる可視化手段を選定したうえで導入する必要がある。

## 目的

シミュレーション結果を高速に可視化する仕組みを追加し、初期位置、障害物、time step ごとのエージェント位置推移をアニメーション的に確認できるようにする。

## スコープ

- 可視化ツール候補の比較と選定
- `matplotlib` を採用しない理由の整理
- 現行 `SimulationResult.history` と obstacle 情報から描画可能な最小データモデルの整理
- 初期位置、障害物、step ごとの agent 位置を表示する viewer または export 手段の実装
- CLI または script から 1 コマンドで可視化を起動できる導線の追加
- 実装と docs の同期

## 非スコープ

- 論文向けの高品位レンダリング
- Web アプリ全体の構築
- proxemic / cnav の専用可視化
- 実時間の双方向シミュレータ編集機能

## 要件

- 障害物形状を agent の軌跡と同時に表示できること
- 初期状態から最終状態までの step 再生ができること
- 100 agent 規模、数十 step 以上でも反復利用しやすい速度であること
- headless に近い export 手段、または軽量なローカル viewer のどちらかを持つこと
- 現行 scenario / simulation データ構造から大きな破壊的変更なしに利用できること

## 検討観点

- 候補は `matplotlib` 以外とする
- Python から扱いやすく、描画性能が高いものを優先する
- 例:
  - `pyqtgraph`
  - `vispy`
  - WebGL / browser-based export
  - `dearpygui`
- 依存追加、保守性、Poetry 環境での再現性も比較対象に含める

## 完了条件

- 可視化ツールの採用理由が docs にある
- 1 つ以上の scenario をアニメーション的に可視化できる
- 障害物と agent 位置推移が同時に見える
- 起動手順が README または仕様書に記録されている
- 最低限の動作確認方法が test または smoke 手順として残っている

## 想定成果物

- `src/cmis_ca/visualization/`
- `scripts/` または `cli/` の可視化導線
- `docs/specifications/` の可視化仕様書
- `README.md`

## 作業メモ

- まずは `SimulationResult.history` を描ける最小 viewer を優先する
- regression helper が返す metric とは別に、再生用フレーム列の抽出 API が必要になる可能性がある
- GUI 依存が重い場合は、軽量 viewer と動画 / HTML export を分けて考える
- 設計段階の採用候補比較は [../architecture/visualization-design.md](../architecture/visualization-design.md) を正とする
- 初回導入では `pyqtgraph` を採用し、`vispy` と `WebGL` 系は後段候補として扱う

## 実施メモ

- `src/cmis_ca/visualization/` に trace model、trace builder、PyQtGraph viewer を追加した
- `cmis-ca visualize` を追加し、scenario 実行から viewer 起動までを 1 コマンド化した
- `pyqtgraph` と `PySide6` は lazy import とし、非 GUI テストが依存なしで通るようにした
- trace builder と CLI 導線を pytest 化し、viewer 表示自体は manual smoke に寄せた
- 実装仕様を [../specifications/pyqtgraph-visualization.md](../specifications/pyqtgraph-visualization.md) に記録した
