#pragma once

// ==============================================
//  User Configuration  — แก้ที่นี่ที่เดียว
// ==============================================

// --- WiFi ---
#define WIFI_SSID        "YourSSID"        // ชื่อ WiFi ที่ต้องการเชื่อมต่อ
#define WIFI_PASS        "YourPassword"    // รหัสผ่าน WiFi

// --- MQTT broker ---
#define MQTT_BROKER      "139.180.132.158"
#define MQTT_PORT        1883
#define MQTT_CLIENT_ID   "lora-receiver-R01"
#define MQTT_TOPIC       "ble/scan"
// Users: DNP0001-DNP0005  Pass: <user>@2o26 (o=lowercase o)
#define MQTT_USER        "DNP0001"
#define MQTT_PASS        "DNP0001@2o26"

// --- NTP ---
#define NTP_SERVER1      "pool.ntp.org"
#define NTP_SERVER2      "time.google.com"
#define NTP_TZ_OFFSET_S  (7 * 3600)   // UTC+7 (Bangkok)
#define NTP_DST_S        0

// --- LoRa parameters (must match sender) ---
#define LORA_FREQUENCY   923.0f
#define LORA_BW          125.0f
#define LORA_SF          7
#define LORA_CR          5
#define LORA_TX_POWER    14
#define LORA_PREAMBLE    8

// --- Heltec V3 SX1262 pins ---
#define SX_NSS   8
#define SX_DIO1  14
#define SX_RST   12
#define SX_BUSY  13
#define SPI_SCK  9
#define SPI_MISO 11
#define SPI_MOSI 10

// --- Heltec V3 OLED (SSD1306 128x64 I2C) ---
#define VEXT_CTRL 36
#define OLED_RST  21
#define OLED_SCL  18
#define OLED_SDA  17

// --- WiFi / MQTT retry ---
#define WIFI_TIMEOUT_MS      15000
#define MQTT_MAX_RETRIES     3
#define MQTT_RETRY_DELAY_MS  2000
