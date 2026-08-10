# ILGA プロジェクト向けエージェントガイド

## プロジェクト識別
- プロジェクト名: ILGA
- 主な構成要素:
  - ILGA-TAG
  - ILGA-LOCATOR
  - VIEWER
- 文中では以下の用語を優先して使う:
  - TAG
  - LOCATOR
  - VIEWER

## 引継ぎ情報
- 現在の到達点と未完了項目は `docs/CURRENT_STATUS.md` を確認する。
- Codex Desktop / CLI の環境構成と確認手順は `docs/CODEX_DESKTOP_HANDOVER.md` を確認する。
- ILGA の仕様判断では、2026-03-16版の共通基準文書を上位の基準とする。
- 実装と基準文書が矛盾する場合は、矛盾を明示し、勝手に仕様を変更しない。

## 基本方針
- Human steers, agent executes. を基本とする。
- 変更は小さく、検証しやすい単位を優先する。
- コード変更前に、対象コンポーネントを明確にする。
  - TAG
  - LOCATOR
  - VIEWER
- 明示的な指示がない限り、無関係な複数コンポーネントを1つの作業で同時変更しない。

## 変更管理
- 実装開始前に、可能であればベースブランチまたはベースコミットを確認する。
- 編集前に以下を明示する。
  - 対象ファイル
  - 差分方式か全置換か
  - 検証方法
- 編集後は以下を報告する。
  - 変更したファイルパス
  - ビルド手順 / テスト手順
  - pull後に確認すべき内容
  - 可能であればコミットURL、難しければ同等の要約

## リポジトリ構成の扱い
- current/ と snapshots/ の境界を尊重する。
- 確認なしに snapshots を現行ソースとみなさない。
- 明示的に指定されていない限り、履歴用ではなく現在の作業対象ソースを優先して編集する。

## 実装方針
- 過度に凝った実装よりも、読みやすく一般的で安定した方法を優先する。
- 新しいヘルパーやフレームワークを導入する前に、既存のプロジェクト規約や実装を再利用できないか確認する。
- TAG、LOCATOR、VIEWER の境界を明確に保つ。
- 新しいスクリプトや補助ツールが必要な場合は、適切な scripts/ または tooling 配下に置き、追加理由を説明する。

## 実行と検証
- 手作業だけの確認よりも、再現可能なスクリプトによる検証を優先する。
- 可能であれば、以下は繰り返し実行できる形にする。
  - build
  - flash
  - log capture
  - smoke test
- ハードウェア接続が必要な作業では、実機アクセス可能であることを確認するまで、実機が使える前提で断定しない。

## ドキュメント方針
- AGENTS.md は短く保ち、地図として機能させる。
- 詳細な設計判断は docs/ に分離する。
- 重要な判断が発生した場合は、必要に応じて以下の作成を提案する。
  - docs/adr/
  - docs/exec-plans/
  - Issue または PR ノート

## コミュニケーション
- 指示が曖昧な場合は、曖昧な点を列挙し、安全側の解釈を提示する。
- 不確実な点を隠さない。
- 仮定を置いた場合は明示する。

## TAG current working base
- TAG migration source snapshot:
  - TAG/zephyr_apps/snapshots/nrf5340dk-bmi270_ble_marge
- TAG active working directory:
  - TAG/zephyr_apps/current/nrf54l15_port
- Do not directly edit the snapshot migration source unless explicitly requested.
- Use the snapshot as a fixed reference, and apply 54L porting work in the active working directory.
- For TAG tasks, prefer:
  - first: build succeeds
  - second: flash succeeds
  - third: boot/log output is confirmed
  - fourth: BMI270 and BLE behavior are restored
