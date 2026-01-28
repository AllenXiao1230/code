Rocket Ground Station
=====================

PyQt5 製作的火箭地面站 GUI，包含序列埠連線、即時遙測顯示、地圖軌跡、高度曲線與 CSV 記錄控制。統一採用 54-byte 固定長度封包（Header=0xAA55）的解碼，並以 T+ 任務時間呈現飛行狀態。


快速開始
--------
1. 安裝需求：Python 3.10+（已測 PyQtWebEngine 需要 Qt WebEngine 依賴）、可用的 serial 裝置。
2. 安裝套件：
   ```bash
   pip install -r requirements.txt
   ```
3. 啟動地面站：
   ```bash
   python main.py
   ```
4. 在「序列埠」面板選擇正確的 Port/波特率（預設 115200）後按「Connect」。


使用指南（介面區塊）
------------------
- 序列埠：掃描可用埠，顯示連線狀態/錯誤，Connect/Disconnect 控制。
- 飛行狀態控制：顯示當前狀態；狀態變更為 ASCENT 時會鎖定任務 T0；IDLE 會重置任務時間。
- 記錄控制：開始/停止 CSV 記錄（檔名 `telemetry_<timestamp>.csv`）。狀態會顯示於按鈕旁並寫入事件面板。
- 速度 / 高度：左右分欄顯示 GPS/氣壓的速度與高度。
- 感測器數據：顯示狀態、錯誤碼、開機時間（秒）、落水狀態、經緯度、偏航、衛星數、Roll/Pitch、AccX/Y/Z、GyroX/Y/Z、溫濕度與氣壓。
- 地圖：Leaflet 地圖（需網路存取 tile），標記當前 GPS 位置並以折線連接歷史點。
- 高度曲線：顯示 30 秒滑動窗口，時間軸隨時間向右移動，最新數據在右端。
- 事件指示器：列出連線、錯誤、記錄狀態等訊息。
- 電壓/時間：電池電壓、任務時間（T+ 格式）。
- 3D 姿態：以 Roll/Pitch/Yaw 即時旋轉火箭模型，座標軸固定在畫面。


遙測封包格式（54 bytes）
----------------------
所有欄位為 little-endian。

| Byte Offset | 長度 | 欄位名稱 | 型態 | 單位 / 說明 |
| ----------- | ---- | -------- | ---- | ----------- |
| 0–1 | 2 | Header | `uint16` | 固定 `0xAA55`（bytes: `0x55 0xAA`） |
| 2 | 1 | MsgType | `uint8` | `0x01` = Telemetry |
| 3–6 | 4 | TimeTag | `uint32` | 開機後 ms |
| 7–10 | 4 | Latitude (GPS) | `int32` | deg × 1e7 |
| 11–14 | 4 | Longitude (GPS) | `int32` | deg × 1e7 |
| 15–16 | 2 | GPS Altitude (GPS) | `int16` | 0.1 m |
| 17–18 | 2 | GPS Speed (GPS) | `int16` | 0.1 m/s |
| 19 | 1 | GPS Sat Count (GPS) | `uint8` | 顆 |
| 20–21 | 2 | Roll (IMU) | `int16` | 0.01 deg |
| 22–23 | 2 | Pitch (IMU) | `int16` | 0.01 deg |
| 24–25 | 2 | Yaw (IMU) | `uint16` | 0.1 deg |
| 26–27 | 2 | GyroX (IMU) | `int16` | 0.1 deg/s |
| 28–29 | 2 | GyroY (IMU) | `int16` | 0.1 deg/s |
| 30–31 | 2 | GyroZ (IMU) | `int16` | 0.1 deg/s |
| 32–33 | 2 | AccX (ADXL) | `int16` | 0.01 g |
| 34–35 | 2 | AccY (ADXL) | `int16` | 0.01 g |
| 36–37 | 2 | AccZ (ADXL) | `int16` | 0.01 g |
| 38–41 | 4 | Baro Pressure (BMP) | `uint32` | Pa（UI/CSV 轉為 kPa 顯示） |
| 42–43 | 2 | Baro Altitude (BMP) | `int16` | 0.1 m |
| 44–45 | 2 | Temperature (SHT) | `int16` | 0.01 °C |
| 46–47 | 2 | Humidity (SHT) | `uint16` | 0.1 %RH |
| 48–49 | 2 | Battery | `uint16` | mV |
| 50 | 1 | FlightState | `uint8` | 0=TEST,1=IDLE,2=PREFLIGHT,3=ASCENT,4=APOGEE,5=DESCENT,6=LANDED,99=ABORT |
| 51 | 1 | ErrorCode | `uint8` | 0=NONE,1=LoRa lost,2=GPS lost,3=IMU fail,4=Baro fail,5=Battery low,6=Sensor timeout,255=Unknown |
| 52 | 1 | WaterDetected | `uint8` | 0=否,1=是 |
| 53 | 1 | CRC8 | `uint8` | XOR(Byte 0–52) |

收到 MsgType=0x01 且 CRC 正確才會更新 UI。TimeTag 會顯示為開機時間（秒），任務時間以 ASCENT 為 T0 顯示 T+。

資料來源與目前實作
----------------
- IMU（ICM42688）：提供姿態融合輸出（Roll/Pitch/Yaw）與 GyroX/Y/Z。
- ADXL375：提供加速度 AccX/Y/Z（高 G 加速度）。
- BMP390：提供氣壓（Pa）與氣壓高度（m）。
- SHT31：提供溫度/濕度。
- GPS：經緯度/速度/高度/衛星數（目前韌體端多數仍為 0，待接入 GPS 串流）。
- Battery：目前韌體端預留（mV），若接入 ADC 需填值。

姿態融合與穩定化
----------------
- 姿態融合：使用 Madgwick IMU（gyro + accel）計算 Roll/Pitch/Yaw。
- Yaw 歸零：啟動後第一筆姿態視為零點（yaw offset）。
- Yaw 鎖定：當 |Pitch| ≥ 80° 時暫停更新 yaw，避免接近垂直的 Euler 亂跳。
- Roll/Pitch 平滑：韌體端低通濾波（目前 `smooth_alpha = 0.2`）。


座標系 / 姿態定義
----------------
- 火箭座標系：Z 軸指向鼻錐/天空，X 軸指向火箭右邊，Y 軸指向發射水平方向（前方）。
- 姿態角定義：Roll 繞 Z 軸、Pitch 繞 X 軸、Yaw 繞 Y 軸。

IMU / ADXL 軸向對應（中文說明）
----------------------------
- ICM42688 原始軸：以晶片座標讀取 `accX/accY/accZ`、`gyrX/gyrY/gyrZ`。
- 姿態融合使用軸：為了符合「Roll=繞Z、Pitch=繞X、Yaw=繞Y」的定義，融合時做了軸重排：  
  - `ax_body = accZ`、`ay_body = accX`、`az_body = accY`  
  - `gx_body = gyrZ`、`gy_body = gyrX`、`gz_body = gyrY`
- ADXL375 對應火箭座標系（加速度輸出）：  
  - `AccX = -ADXL_Y`、`AccY = ADXL_X`、`AccZ = ADXL_Z`
- Yaw 校正：啟動後第一筆姿態會作為零點（yaw 歸零）。
- Yaw 鎖定：俯仰接近垂直時（|Pitch| ≥ 80°）暫停更新 yaw，避免數值亂跳。


CSV 記錄
--------
- 檔名：`telemetry_<unix_ts>.csv`
- 欄位：`time, lat, lon, alt, gps_alt, baro_alt, speed, heading, sat, roll, pitch, accx, accy, accz, gyro_x, gyro_y, gyro_z, battery, temp, hum, pressure_kpa, status, water`
- 操作：在「記錄控制」點「開始記錄」啟動，停止後自動關檔。


重播（開發用途）
---------------
`replay.py` 提供 `ReplayWorker`，可將 (timestamp, raw_bytes) 清單以原速或倍速餵入序列處理流程，模擬序列資料來源。可在開發自訂入口，透過 `data_emitted` 連接到 `MainWindow._on_serial_data` 或 `_process_legacy_stream`。


程式結構
-------
- `main.py`：Qt 入口，啟動 `MainWindow`。
- `gui_main.py`：UI 布局與事件處理、封包解碼、資料分發、CSV 記錄控制。
- `gui_widgets.py`：速度/高度、遙測面板、記錄控制等組件。
- `serial_comm.py`：序列埠背景讀取執行緒。
- `state_machine.py`：飛行狀態、錯誤碼定義與訊號。
- `plot_panel.py`：高度曲線（滑動窗口）。
- `map_view.py`：Leaflet 地圖與軌跡線。
- `logger.py`：CSV 記錄器。


開發與注意事項
-------------
- 地圖需要網路下載 OSM tile；無網路時地圖空白但其他功能仍可用。
- 目前僅解碼 54-byte 固定封包；若韌體改版需同步更新 `gui_main.py` 的 `_handle_legacy_frame`。
- 任務時間以 ASCENT 為 T0；切回 IDLE 會重置 T0。
- 預設序列埠/波特率：`COM3`、115200，可在 UI 選擇或修改 `config.py`。
