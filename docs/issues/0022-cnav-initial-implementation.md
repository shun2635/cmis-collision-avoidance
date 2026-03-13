# Issue 0022: CNav 初期実装の親 issue を整理する

- ステータス: completed
- 優先度: medium
- 関連文書:
  - [docs/algorithms/cnav.md](../algorithms/cnav.md)
  - [docs/architecture/api.md](../architecture/api.md)
  - [docs/architecture/repository-structure.md](../architecture/repository-structure.md)
  - [docs/specifications/python-skeleton-detailed-design.md](../specifications/python-skeleton-detailed-design.md)
  - [README.md](../../README.md)

## 背景

このリポジトリでは ORCA を共通基盤の最初の基準実装として整備してきた。  
その次段として、`局所衝突回避としての ORCA` の上に `分散協調による intended velocity 選択` を載せる CNav を追加したい。

一方で CNav は、単に ORCA の別パラメータ版ではない。  
neighbor の intended velocity 通信、constrained neighbor ranking、短期 simulation による action 評価、reward による action 選択など、ORCA にはない上位レイヤを持つ。  
また action-selection の系譜は ALAN に接続しており、そこを曖昧にしたまま実装すると論文由来の境界が崩れる。

そのため、まず出典の使い分けを固定し、次に current architecture に沿って `どこまでを core に残し、どこからを algorithms/cnav に置くか` を明文化したうえで初期実装へ進む必要がある。

## 目的

2020 C-Nav 論文を正本として、CNav の action-selection 層を既存 ORCA 実装の上に追加するための親 issue を整理する。  
個別の実装作業は後続 issue へ分割し、この issue では採用方針、初期スコープ、実施順を固定する。

## 決定事項

- ORCA 再利用は `per-agent solve の helper 抽出` を採用する
- 初期実装スコープは論文既定 action set、固定 `T=2`、goal 必須、algorithm 内部 cache 通信に限定する
- CNav 実装は umbrella issue ではなく、実装単位の個別 issue へ分けて進める

## スコープ

- CNav の理論整理文書を追加する
- 原典、詳細版、ALAN の役割分担を docs に明記する
- ORCA 再利用方針と初期実装スコープを docs に固定する
- 後続 issue を実装順に分割して管理できる状態にする

## 非スコープ

- CNav 本体コードの実装
- ALAN の bandit learning の実装
- 通信遅延、通信損失、通信帯域の詳細モデル
- 論文内の全 scenario の一括再現
- 物理ロボット統合
- proxemic の同時実装

## 要件

- ORCA 再利用方針が明文化されていること
- 初期実装のスコープ制限が明文化されていること
- 実装順の個別 issue が切られていること
- docs 間で矛盾がないこと

## 完了条件

- `docs/algorithms/cnav.md` に採用方針と初期スコープが記録されている
- CNav 初期実装の作業が個別 issue に分かれている
- `docs/issues/README.md` から追える

## ドキュメント構造

- 実装前
  - `docs/algorithms/cnav.md`
    - 理論、出典、実装境界、ALAN との差分
- 実装中
  - 本 issue
    - タスク分解、完了条件、設計上の注意
- 実装後
  - `docs/specifications/` に CNav 初期仕様書を追加
  - `README.md` に CLI 導線と対応状況を反映

## 後続 issue

1. [0023-cnav-orca-per-agent-solve-helper.md](0023-cnav-orca-per-agent-solve-helper.md)
   - ORCA の per-agent solve を helper として切り出す
2. [0024-cnav-action-model-and-intent-cache.md](0024-cnav-action-model-and-intent-cache.md)
   - parameter、action set、intent cache、algorithm 雛形を追加する
3. [0025-cnav-coordination-evaluation.md](0025-cnav-coordination-evaluation.md)
   - constrained neighbor ranking、SimMotion、reward を実装する
4. [0026-cnav-cli-scenarios-tests-and-spec.md](0026-cnav-cli-scenarios-tests-and-spec.md)
   - registry、CLI、scenario、tests、仕様書を仕上げる

## 想定成果物

- `docs/algorithms/cnav.md`
- `docs/issues/0023-cnav-orca-per-agent-solve-helper.md`
- `docs/issues/0024-cnav-action-model-and-intent-cache.md`
- `docs/issues/0025-cnav-coordination-evaluation.md`
- `docs/issues/0026-cnav-cli-scenarios-tests-and-spec.md`

## 作業メモ

- 実装そのものは後続 issue で進める
- 初回実装では、論文どおり `T=2` と固定 action 集合を優先し、一般化は後段に回す
- 現行 `Simulator` は step 前に goal から `preferred_velocity` を更新するが、CNav ではこれを `goal-oriented baseline` として使い、最終的な `v_intent` は algorithm 内部状態で保持する
- goal が未設定の scenario では constrained neighbor ranking が定義しにくいので、初回実装では `全 agent に goal_position 必須` とする
- simulation-based 評価では self と neighbor の一時 state が必要になるため、runtime 専用の軽量 dataclass を `algorithms/cnav/` 側に閉じ込める
- 可視化比較は初期実装の完了条件に含めない

## 実施メモ

- issue `0023` で ORCA の per-agent solve helper を抽出した
- issue `0024` で CNav の parameter、action set、intent cache、algorithm 雛形を追加した
- issue `0025` で constrained neighbor ranking と short-horizon action evaluation を実装した
- issue `0026` で registry、CLI、scenario、tests、仕様書、README を更新し、初期導入を完了した
