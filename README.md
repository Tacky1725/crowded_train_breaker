# 満員電車ブレイカー
## 傾き制御プログラム
### スケッチ
- actuator_dual: アクチュエータの制御
- actuator_feedback: 傾きのフィードバック制御プログラム
- read_voltage: 可変抵抗で傾きをアナログ入力で取得する

## ESP32-DevKitC-32E へのインポート
### ピン配置
- https://www.farmsoft.jp/1274/
- https://docs.espressif.com/projects/esp-dev-kits/en/latest/esp32/esp32-devkitc/user_guide.html#functional-description

### CP210xドライバを入れる（Windows）

- Silicon Labs の CP210x VCP Drivers をインストール。
[silabs.com](https://www.silabs.com/software-and-tools/usb-to-uart-bridge-vcp-drivers?tab=downloads)から"CP210x VCP Windows"を選択

- インストール後にUSBを挿し直し、デバイスマネージャ → ポート (COMとLPT) に
"Silicon Labs CP210x USB to UART Bridge (COMX)" が出るか確認。

### USB-TypeBでインポート
つながらないときは
- uploading前にBOOTボタン長押し
- コンソールに"Connecting..."と表示されたらBOOTボタン解放

もしくは
- BOOT を押しながら EN/RESET を1回押し
- 1秒後に BOOT を離してから書き込み（手動でDLモード）