# BLE Receiver — Heltec WiFi LoRa 32 V3

รับ packet LoRa จาก BLE Scanner node แล้ว forward ข้อมูลไปยัง MQTT broker  
ใช้สถาปัตยกรรม FreeRTOS dual-core เพื่อให้ LoRa รับสัญญาณได้ต่อเนื่องขณะที่ WiFi/MQTT กำลัง reconnect

---

## ภาพรวมระบบ

```
[BLE Scanner Node] --LoRa--> [BLE Receiver (this device)] --MQTT--> [Broker]
```

| บทบาท | Task | Core | Priority |
|---|---|---|---|
| รับ LoRa, parse payload, ส่ง ACK | `loraTask` | 0 | 3 |
| WiFiManager, NTP, MQTT publish | `wifiTask` | 1 | 2 |
| ตรวจ BOOT button | `loop()` | 1 | 1 |

ข้อมูลแต่ละ device ถูกส่งผ่าน FreeRTOS Queue (ขนาด 35 items) จาก LoRa Task ไปยัง WiFi Task

---

## Hardware

- **บอร์ด:** Heltec WiFi LoRa 32 V3 (ESP32-S3 + SX1262)
- **จอ:** SSD1306 OLED 128×64 (I2C)

### ขา Pin

| ฟังก์ชัน | GPIO |
|---|---|
| SX1262 NSS | 8 |
| SX1262 DIO1 | 14 |
| SX1262 RST | 12 |
| SX1262 BUSY | 13 |
| SPI SCK | 9 |
| SPI MISO | 11 |
| SPI MOSI | 10 |
| OLED RST | 21 |
| OLED SCL | 18 |
| OLED SDA | 17 |
| Vext Control | 36 |
| BOOT Button | 0 |

---

## OLED Display

```
== BLE Receiver ==
─────────────────
LoRa: Listening #12
Node: S01 -55dBm
IP  : 192.168.1.42
MQTT: OK +5
```

---

## LoRa Parameters

ต้องตรงกับฝั่ง Sender ทุกค่า

| พารามิเตอร์ | ค่า |
|---|---|
| Frequency | 923.0 MHz |
| Bandwidth | 125 kHz |
| Spreading Factor | 7 |
| Coding Rate | 5 (4/5) |
| TX Power | 14 dBm |
| Preamble | 8 |

---

## Binary Packet Format

| Byte | ความหมาย |
|---|---|
| 0 | Magic byte `0xBE` |
| 1 | Packet index (0-based, uint8) |
| 2 | Total packets in burst (uint8) |
| 3 | NODE_ID length |
| 4–6 | NODE_ID (3 bytes) |
| 7 | Device count in this packet (uint8) |
| 8–9 | Total devices in scan cycle (uint16, little-endian) |
| 10+ | Device entries: 6 bytes MAC + 1 byte RSSI (ซ้ำ N ครั้ง) |

`DEVS_PER_PKT = 35` (จำนวน device สูงสุดต่อ packet)

เมื่อรับ packet ได้สำเร็จ บอร์ดจะส่ง ACK กลับ:
```json
{"ack":1,"gw":"R01"}
```

---

## MQTT

| พารามิเตอร์ | ค่า |
|---|---|
| Broker | `139.180.132.158:1883` |
| Client ID | `lora-receiver-R01` |
| Topic | `ble/scan` |

Payload ที่ publish (JSON):
```json
{
  "ts": "2026-04-23 10:30:00",
  "mac": "AABBCCDDEEFF",
  "rssi": -72,
  "no": "36/70"
}
```

- **`no`** — ลำดับ device แบบ 1-based ต่อเนื่องข้ามทุก packet / จำนวน device ทั้งหมดใน scan cycle
  - คำนวณจาก: `(pkt_index × 35) + local_index + 1`

---

## WiFi Configuration (WiFiManager)

1. เปิดบอร์ดครั้งแรก (หรือยังไม่มี WiFi credentials) → บอร์ดเปิด Access Point ชื่อ **`BLE-Receiver`** รหัส **`12345678`**
2. เชื่อมต่อด้วยโทรศัพท์หรือคอมพิวเตอร์แล้วเปิด `192.168.4.1`
3. เลือก WiFi และกรอก password → บันทึกลง flash อัตโนมัติ
4. **ล้าง WiFi credentials:** กดปุ่ม BOOT (GPIO0) ค้างไว้ขณะเปิดเครื่อง 3 วินาที

---

## Dependencies (PlatformIO)

```ini
lib_deps =
    jgromes/RadioLib @ ^6.6.0
    bblanchon/ArduinoJson @ ^6.21.5
    olikraus/U8g2 @ ^2.35.7
    knolleary/PubSubClient @ ^2.8.0
    tzapu/WiFiManager @ ^2.0.17
```

---

## การ Build และ Upload

```bash
# Build
pio run

# Upload + เปิด Serial Monitor
pio run --target upload ; pio device monitor
```

Serial baud rate: **115200**

---

## หมายเหตุ

- NTP timezone: UTC+7 (Bangkok)
- ถ้า MQTT broker ไม่พร้อม ข้อมูลจะถูก buffer ไว้ใน queue (สูงสุด 35 items) และ publish เมื่อ broker กลับมา
- LoRa รับสัญญาณได้ตลอดแม้ WiFi/MQTT จะ reconnect อยู่
