# 可視化基盤の設計

## 1. 目的

この文書は、issue `0021-fast-simulation-visualization` に対して、高速可視化基盤の採用候補を比較し、初回導入で採る技術と構成を固定する。

対象要件は以下。

- 初期位置、障害物、time step ごとの agent 位置推移をアニメーション的に見たい
- `matplotlib` は使わない
- 100 agent 規模、数十 step 以上でも反復確認しやすい速度が必要
- 現行 `Scenario` と `SimulationResult.history` を大きく壊したくない

## 2. 候補

今回比較する候補は以下の 3 系統とする。

1. `pyqtgraph`
2. `vispy`
3. `WebGL / browser-based`
   初回比較では Python から扱いやすい代表として Plotly WebGL 系を基準にする

## 3. 比較結果

| 候補 | 長所 | 短所 | 判定 |
| --- | --- | --- | --- |
| `pyqtgraph` | Python からそのまま使いやすい。Qt widget ベースで 2D viewer を最短で組める。対話ウィンドウを command line から起動しやすい。 | Qt 依存がある。browser export は主目的ではない。GPU shader 前提ではない。 | 採用 |
| `vispy` | GPU / OpenGL 前提で高性能。2D/3D 両対応で将来拡張性が高い。 | OpenGL と backend 要件が重い。driver 依存の切り分けが増える。今回の 2D viewer 要件にはやや過剰。 | 今回は不採用 |
| `WebGL / Plotly` | HTML export がしやすい。browser 共有が容易。WebGL trace で大規模点群に強い。 | browser 前提になる。アニメーションは `scatter` / `bar` 系中心で、viewer の制御自由度が低い。Dash や HTML 生成を含めると構成が重い。 | 今回は不採用 |

## 4. 採用判断

初回導入では `pyqtgraph` を採用する。

理由は以下。

1. 今回必要なのは `2D の障害物 + agent 点群 + step 再生` であり、3D や shader 表現は不要
2. `SimulationResult.history` をそのまま受けてローカル viewer を開く実装が最短で済む
3. CLI / script から 1 コマンドで viewer を起動する導線を作りやすい
4. `vispy` ほど backend / OpenGL 切り分けが重くない
5. `WebGL` 系ほど browser / HTML 側の状態管理を持ち込まずに済む

## 5. 不採用理由

### 5.1 `vispy`

`vispy` は高性能だが、今回の要件では性能よりも `導入の軽さ` と `研究室内での再現性` が重要である。  
公式ドキュメントでも、VisPy は OpenGL とウィンドウ backend を必要とし、GPU と driver 条件を強く受ける。  
今回の可視化は 2D で十分なので、まずは `pyqtgraph` の方が筋が良い。

将来、以下が必要になったら再検討する。

- 1000 体以上の常時再生
- より高密度な軌跡描画
- GPU shader ベースの表現
- 3D 可視化

### 5.2 `WebGL / Plotly`

WebGL 系は HTML export に強いが、今回の主目的は `ローカルで速く再生しながらデバッグする viewer` である。  
Plotly 公式 docs でも、WebGL trace は高速だが browser と GPU 条件に依存し、アニメーションの smooth transition も制約がある。  
研究室内のローカル検証導線としては、初回導入の複雑さが高い。

ただし、共有用 artifact としての HTML export 需要はあり得るため、`phase 2` 以降の export backend 候補としては残す。

## 6. 採用構成

### 6.1 方針

初回は `軽量なローカル viewer` を先に作る。  
動画 export や HTML export は後段へ分離する。

### 6.2 想定モジュール

- `src/cmis_ca/visualization/models.py`
  - `VisualizationTrace`
  - `VisualizationFrame`
  - `ObstaclePrimitive`
- `src/cmis_ca/visualization/trace_builder.py`
  - `Scenario` と `SimulationResult` から可視化用フレーム列を生成
- `src/cmis_ca/visualization/pyqtgraph_viewer.py`
  - `pyqtgraph` による 2D viewer
- `src/cmis_ca/visualization/run_visualization.py`
  - CLI / script から呼ぶ起動導線

### 6.3 表示要素

- 障害物
  - polygon / open chain を線分列で描画
- agent
  - 現在位置を scatter で描画
- 軌跡
  - 必要に応じて直近 N step または全履歴を polyline で描画
- 初期位置
  - 現在位置と色や透過で区別して表示

### 6.4 再生制御

- play / pause
- step 前進 / step 後退
- スライダで frame 移動
- 再生速度変更
- view fit / zoom / pan

## 7. データ境界

可視化層は ORCA の計算本体に入れない。  
入力は以下に限定する。

- `Scenario`
- `SimulationResult`

これにより、将来 `proxemic` や `cnav` を追加しても、同じ trace builder を再利用しやすくする。

## 8. CLI 方針

初回導入では次のいずれかを採る。

1. `cmis-ca visualize --scenario ... --algorithm orca`
2. `poetry run python scripts/visualize_scenario.py --scenario ...`

初回は実装コストを抑えるため、script でもよい。  
ただし最終的には `cmis-ca visualize` へ寄せる。

## 9. テスト方針

GUI の完全自動テストは重いため、初回は以下で十分とする。

- trace builder の unit test
- 可視化用データ件数の smoke test
- viewer 起動手順の manual smoke を docs に記録

## 10. 今回の設計結論

- 採用: `pyqtgraph`
- 位置付け: `ローカル高速 viewer`
- 後段候補:
  - `Plotly / WebGL` は共有用 export backend 候補
  - `vispy` は高性能化が本当に必要になった時の再検討候補

## 11. 参考

- PyQtGraph documentation:
  - <https://pyqtgraph.readthedocs.io/en/latest/getting_started/how_to_use.html>
  - <https://pyqtgraph.readthedocs.io/en/latest/getting_started/installation.html>
  - <https://pyqtgraph.readthedocs.io/en/latest/api_reference/graphicsItems/plotdataitem.html>
- PyQtGraph OpenGL note:
  - <https://pyqtgraph.readthedocs.io/en/pyqtgraph-0.13.4/api_reference/3dgraphics/>
- VisPy documentation:
  - <https://vispy.org/>
  - <https://vispy.org/installation.html>
- Plotly documentation:
  - <https://plotly.com/python/performance/>
  - <https://plotly.com/python/interactive-html-export/>
  - <https://plotly.com/python/animations/>
