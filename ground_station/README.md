Rocket Ground Station
=====================

PyQt5 製作的火箭地面站 GUI，包含序列埠連線、即時遙測顯示、地圖軌跡、高度曲線與 CSV 記錄控制。支援 41-byte 固定長度封包（0xAA55 開頭）的解碼，並以 T+ 任務時間呈現飛行狀態。


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
- 感測器數據：顯示狀態、錯誤碼、開機時間（秒）、任務時間（T+）、經緯度、航向、衛星數、AccZ、俯仰、陀螺、溫濕度與氣壓。
- 地圖：Leaflet 地圖（需網路存取 tile），標記當前 GPS 位置並以折線連接歷史點。
- 高度曲線：顯示 30 秒滑動窗口，時間軸隨時間向右移動，最新數據在右端。
- 事件指示器：列出連線、錯誤、記錄狀態等訊息。
- 電壓/時間：電池電壓、任務時間（T+ 格式）。


遙測封包格式（41 bytes）
----------------------
所有欄位為 little-endian。

| Byte Offset | 長度 | 欄位名稱 | 型態 | 單位 / 說明 |
| ----------- | ---- | -------- | ---- | ----------- |
| 0–1 | 2 | Header | `uint16` | 固定 `0xAA55` |
| 2 | 1 | MsgType | `uint8` | `0x01` = Telemetry |
| 3–6 | 4 | TimeTag | `uint32` | 開機後 ms |
| 7–10 | 4 | Latitude | `int32` | deg × 1e7 |
| 11–14 | 4 | Longitude | `int32` | deg × 1e7 |
| 15–16 | 2 | GPS Altitude | `int16` | 0.1 m |
| 17–18 | 2 | GPS Speed | `int16` | 0.1 m/s |
| 19–20 | 2 | GPS Heading | `uint16` | 0.1 deg |
| 21 | 1 | GPS Sat Count | `uint8` | 顆 |
| 22–23 | 2 | AccZ | `int16` | 0.01 g |
| 24–25 | 2 | Pitch | `int16` | 0.01 deg |
| 26–27 | 2 | GyroZ | `int16` | 0.1 deg/s |
| 28 | 1 | FlightState | `uint8` | 0=TEST,1=IDLE,2=PREFLIGHT,3=ASCENT,4=APOGEE,5=DESCENT,6=LANDED,99=ABORT |
| 29 | 1 | ErrorCode | `uint8` | 0=NONE,1=LoRa lost,2=GPS lost,3=IMU fail,4=Baro fail,5=Battery low,6=Sensor timeout,255=Unknown |
| 30–31 | 2 | Battery | `uint16` | mV |
| 32–33 | 2 | Temperature | `int16` | 0.01 °C |
| 34–35 | 2 | Humidity | `uint16` | 0.1 %RH |
| 36–39 | 4 | Baro Pressure | `uint32` | Pa |
| 40 | 1 | CRC8 | `uint8` | XOR(Byte 0–39) |

收到 MsgType=0x01 且 CRC 正確才會更新 UI。TimeTag 會顯示為開機時間（秒），任務時間以 ASCENT 為 T0 顯示 T+。


CSV 記錄
--------
- 檔名：`telemetry_<unix_ts>.csv`
- 欄位：`time, lat, lon, alt, speed, heading, sat, accz, pitch, gyro, battery, temp, hum, pressure, status`
- 操作：在「記錄控制」點「開始記錄」啟動，停止後自動關檔。


重播（開發用途）
---------------
`replay.py` 提供 `ReplayWorker`，可將 (timestamp, raw_bytes) 清單以原速或倍速餵入解析器，模擬序列資料來源。可在開發自訂入口，透過 `data_emitted` 連接到 `ProtocolParser.feed`。


程式結構
-------
- `main.py`：Qt 入口，啟動 `MainWindow`。
- `gui_main.py`：UI 布局與事件處理、封包解碼、資料分發、CSV 記錄控制。
- `gui_widgets.py`：速度/高度、遙測面板、記錄控制等組件。
- `serial_comm.py`：序列埠背景讀取執行緒。
- `protocol.py`：SYNC(0xAA55) 長度制封包解析器（新協定保留）。
- `state_machine.py`：飛行狀態、錯誤碼定義與訊號。
- `plot_panel.py`：高度曲線（滑動窗口）。
- `map_view.py`：Leaflet 地圖與軌跡線。
- `logger.py`：CSV 記錄器。


開發與注意事項
-------------
- 地圖需要網路下載 OSM tile；無網路時地圖空白但其他功能仍可用。
- 目前主要解碼 41-byte 固定封包；若韌體改版需同步更新 `gui_main.py` 的 `_handle_legacy_frame`。
- 任務時間以 ASCENT 為 T0；切回 IDLE 會重置 T0。
- 預設序列埠/波特率：`COM3`、115200，可在 UI 選擇或修改 `config.py`。
