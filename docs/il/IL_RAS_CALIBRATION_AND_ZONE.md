# IL RAS 校正・近距離ゾーン判定 暫定設計

更新日: 2026-09-28

## 1. 対象範囲

本設計は、nRF54L15 DK 2台とNordic RAS `cs_de`の公式位相傾き値を使った0.5～1.5 m技術デモを対象とする。Phase 1の2 m enter / 3 m exitを確定するものではなく、人体遮蔽、向き変更、動的横断、1.5 m超は未検証である。

## 2. 処理順

1. 公式位相傾き値を取得する。
2. 直近5フレームの中央値を求める。
3. 設置時の0.5 m / 1.5 mアンカーから求めたgainとoffsetで距離へ変換する。
4. 取得安定性と校正有効性を確認する。
5. 0.75 m以下で`inside`、1.25 m以上で`outside`、その間は直前状態を保持する。

0.75～1.25 mの0.50 m幅は、保存済み2系列で確認した位相方式の系列間最大誤差0.228 mの約2倍より大きくするために設定した。0.5 mアンカーがenter側、1.5 mアンカーがexit側へ入ることも両系列で確認できる。製品境界ではなく技術デモ用の暫定値である。

## 3. 校正ファイル

`scripts/il_cs_ras_calibrate.py create`がJSONを生成する。主な項目は次のとおり。

- `schema_version`: 形式版
- `site_id`, `locator_id`, `tag_id`: 校正を適用できる設置と個体
- `estimator`: `firmware_phase_slope_m`固定
- `window`: 5フレーム移動中央値
- `calibration`: 0.5 / 1.5 mの実距離、測定位相、gain、offset、元CSV情報
- `quality`: 窓出力SD、アンカー誤差、good tone数の暫定基準
- `zone`: enter / exit境界、初期状態、適用範囲

LOCATORの位置・向き、TAG個体、アンテナ条件、周囲の大型物体が変わった場合は、既存ファイルを無条件で流用しない。校正ファイルは測定結果と同じ試験IDで管理する。

## 4. 設置時の2点校正

1. LOCATORを最終設置位置へ固定し、以後動かさない。
2. TAGを正対・同じ高さ・見通しの0.5 mへ置き、安定後に45秒以上取得する。
3. TAGを1.5 mへ移動し、同条件で45秒以上取得する。
4. 両ログを診断解析して`procedures.csv`を生成する。
5. 次で校正JSONを作る。

```bash
python3 scripts/il_cs_ras_calibrate.py create \
  --low-csv analysis_out/il_cs_ras_debug/<LOW>/procedures.csv \
  --low-distance-m 0.5 \
  --high-csv analysis_out/il_cs_ras_debug/<HIGH>/procedures.csv \
  --high-distance-m 1.5 \
  --site-id <SITE_ID> --locator-id <LOCATOR_ID> --tag-id <TAG_ID> \
  --output analysis_out/il_cs_ras_debug/<CALIBRATION_ID>/calibration.json
```

次の場合は校正ファイルを作らず、配置とログ品質を確認する。

- 公式位相値が0.5 mから1.5 mへ増加しない。
- 5フレーム中央値系列の標準偏差が0.20 mを超える。
- good tone数の中央値が60未満。

## 5. アンカー確認

移設後は0.5 mまたは1.5 mの既知距離を1点取得し、次で確認する。

```bash
python3 scripts/il_cs_ras_calibrate.py verify \
  --config analysis_out/il_cs_ras_debug/<CALIBRATION_ID>/calibration.json \
  --anchor-csv analysis_out/il_cs_ras_debug/<VERIFY_ID>/procedures.csv \
  --known-distance-m 0.5 \
  --output analysis_out/il_cs_ras_debug/<VERIFY_ID>/anchor_verification.json
```

補正後誤差0.20 m以下、窓出力SD 0.20 m以下、good tone中央値60以上をすべて満たせば有効とする。1項目でも外れた場合はゾーン判定を開始せず、2点校正をやり直す。これらは今回の6試行用の暫定基準であり、製品仕様ではない。

## 6. ゾーン判定

```bash
python3 scripts/il_cs_ras_calibrate.py classify \
  --config analysis_out/il_cs_ras_debug/<CALIBRATION_ID>/calibration.json \
  --input-csv analysis_out/il_cs_ras_debug/<TRIAL>/procedures.csv \
  --output-csv analysis_out/il_cs_ras_debug/<TRIAL>/zone_states.csv
```

初期状態は`outside`とする。5フレーム中央値自体が遷移を遅らせるため、現段階では追加の連続回数条件を設けない。動的横断ログで検知遅延とチャタリングを確認してから決める。

## 7. 未確定事項

- 5フレームが実時間で何秒になるかは、通常ファームへ組み込む際のprocedure周期で確定する。
- 0.20 m基準、good tone 60、0.75 / 1.25 m境界は保存済み静止データだけによる暫定値である。
- 人体遮蔽、向き、移動速度、複数TAG、温度・長時間ドリフトは未評価である。
- 2 m enter / 3 m exitのPhase 0仮境界へ拡張するには、1.5 m超の校正・静止・横断データが必要である。
