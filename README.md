# DIY Moco Camera Crane

## 概要
**DIY Electric Camera Crane** は、個人や小規模な制作チームが手頃な価格で高品質な電動カメラクレーンを製作・使用できるようにすることを目的としたプロジェクトです。本プロジェクトでは、オープンソースの設計図やソースコードを提供し、必要な材料や手順を公開しています。

このプロジェクトの利点:
- **低コスト**: 制作コストを約20万円以内に抑えた設計。
- **カスタマイズ性**: 使用するカメラや撮影スペースに応じてサイズや仕様を調整可能。
- **オープンソース**: BlenderやTouchDesignerといった無料のソフトウェアで制御を実現。

---

## 特徴
- **フレーム構成**: 軽量かつ高剛性なアルミフレームを使用。
- **モーター制御**: 5軸のステッピングモーターでパン、チルト、上下、左右回転、フォーカスを制御。
- **Blender活用**: Pythonスクリプトを使用してリアルタイム制御と3D空間の同期を実現。
- **ゲームパッド操作**: 直感的な操作性を提供。
- **リアルタイム3D合成**: TouchDesignerとの連携でトラッキング機器不要の合成を可能に。

---

## 動作デモ
![動作デモ](assets/demo_video.mp4)

---

## 必要な材料・部品
- **フレーム**: アルミフレーム（カスタム長さ）
- **ステッピングモーター**: 5個
- **マイコン**: ESP32
- **ベアリング**: 耐荷重仕様の回転テーブル用ベアリング
- **ベルトおよびギア**: 3Dプリントパーツを含む
- **ソフトウェア**: Blender（無料）、TouchDesigner（無料版対応）

詳細な部品リストや購入リンクは [documentation](./docs/parts_list.md) を参照してください。

---

## プロジェクトファイル
リポジトリ内の`projects/`ディレクトリには、以下のプロジェクトファイルが含まれています。

- `blender/`: Blenderプロジェクトファイル（`camera_crane.blend`）
- `touchdesigner/`: TouchDesignerプロジェクトファイル（`crane_control.toe`）

これらのファイルは、クレーン制御のシミュレーションやリアルタイム3D合成に使用します。

---

## セットアップ手順
1. **フレームの組み立て**
   - 設計図に基づき、アルミフレームを組み立てます。

2. **モーターと配線**
   - ステッピングモーターをフレームに取り付け、ベルトで駆動系を構築します。
   - ESP32と各モーターを接続します。

3. **ソフトウェアのセットアップ**
   - Blenderでプロジェクトファイルを読み込みます。
   - TouchDesignerでプロジェクトファイルを読み込み、カメラ位置データをリアルタイムで受信可能にします。

4. **テストと調整**
   - Blenderで制御する仮想クレーンの動作と、現実のクレーンの同期を確認します。
   - 必要に応じてパラメータを調整します。

---

## 使用例
- 映画や動画撮影の特殊効果制作
- ライブ配信でのリアルタイム3D合成
- 低コストでの映像制作環境の構築

---

## ライセンス
このプロジェクトは [MIT License](./LICENSE) の下で公開されています。詳細についてはLICENSEファイルをご覧ください。

---
