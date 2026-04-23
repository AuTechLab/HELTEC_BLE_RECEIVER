/**
 * BLE_Reciver  Heltec WiFi LoRa 32 V3
 *
 * RTOS Architecture  (LoRa independent from WiFi):
 *   LoRa Task  (Core 0, prio 3): DIO1 ISR → readData → ACK → parse → xQueueSend
 *   WiFi Task  (Core 1, prio 2): WiFi.begin → NTP → MQTT → xQueueReceive → publish
 *   loop()     (Core 1, prio 1): idle
 *
 * LoRa keeps receiving even while WiFi/MQTT is reconnecting.
 * Parsed device payloads are held in a FreeRTOS queue (MQTT_QUEUE_SIZE items)
 * and published as soon as the broker is reachable.
 *
 * OLED layout (SSD1306 128x64):
 *   == BLE Receiver ==
 *   LoRa: Listening #3
 *   Node: S01 -55dBm
 *   IP  : 192.168.x.x
 *   MQTT: OK +5
 *
 * LoRa pins (Heltec V3): NSS=8 DIO1=14 RST=12 BUSY=13
 * SPI pins             : SCK=9 MISO=11 MOSI=10
 * OLED pins            : RST=21 SCL=18 SDA=17
 */

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <U8g2lib.h>
#include <RadioLib.h>
#include <ArduinoJson.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include "config.h"
#include <time.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>

// --- ACK sent back to sender ---
static const char ACK_PAYLOAD[] = "{\"ack\":1,\"gw\":\"R01\"}";

// --- Binary packet format (must match sender) ---
#define BIN_MAGIC      0xBE
#define BIN_HEADER_LEN 10
#define DEVS_PER_PKT   35
#define BYTES_PER_DEV  7    // 6 MAC + 1 RSSI

// ==============================================
//  RTOS objects
// ==============================================
#define MQTT_QUEUE_SIZE  35   // max queued payloads

struct MqttItem {
    char     payload[256];
    uint16_t len;
};

static QueueHandle_t     xMqttQueue    = nullptr;
static SemaphoreHandle_t xDisplayMutex = nullptr;

// ==============================================
//  Hardware objects
// ==============================================
SX1262    radio   = new Module(SX_NSS, SX_DIO1, SX_RST, SX_BUSY);
U8G2_SSD1306_128X64_NONAME_F_HW_I2C display(U8G2_R0, OLED_RST, OLED_SCL, OLED_SDA);
WiFiClient   wifiClient;
PubSubClient mqtt(wifiClient);

// --- RX interrupt flag ---
volatile bool rxReady = false;
IRAM_ATTR void onRxDone() { rxReady = true; }

// --- Display state ---
static char     sLoRa[22] = "Init";
static char     sNode[16] = "---";
static char     sRSSI[12] = "";
static char     sWiFi[24] = "Disconnected";
static char     sMQTT[22] = "---";
static uint32_t rxCount   = 0;

// ==============================================
//  OLED
// ==============================================
static void initOledPowerAndReset() {
    // OLED power is supplied by Vext on this board, LOW means ON.
    pinMode(VEXT_CTRL, OUTPUT);
    digitalWrite(VEXT_CTRL, LOW);
    delay(30);

    pinMode(OLED_RST, OUTPUT);
    digitalWrite(OLED_RST, LOW);
    delay(20);
    digitalWrite(OLED_RST, HIGH);
    delay(20);
}

void oledUpdate() {
    if (xDisplayMutex == nullptr) return;
    if (xSemaphoreTake(xDisplayMutex, pdMS_TO_TICKS(200)) != pdTRUE) return;

    char tmp[32];
    display.clearBuffer();
    display.setFont(u8g2_font_6x10_tf);

    display.drawStr(0, 10, "== BLE Receiver ==");
    display.drawHLine(0, 12, 128);

    snprintf(tmp, sizeof(tmp), "LoRa: %s #%lu", sLoRa, (unsigned long)rxCount);
    display.drawStr(0, 24, tmp);

    snprintf(tmp, sizeof(tmp), "Node: %s %s", sNode, sRSSI);
    display.drawStr(0, 36, tmp);

    display.drawStr(0, 48, sWiFi);

    if (strncmp(sMQTT, "Open", 4) == 0) {
        display.drawStr(0, 62, sMQTT);
    } else {
        snprintf(tmp, sizeof(tmp), "MQTT: %s", sMQTT);
        display.drawStr(0, 62, tmp);
    }

    display.sendBuffer();
    xSemaphoreGive(xDisplayMutex);
}

// ==============================================
//  NTP - get ISO-8601 UTC timestamp
//  e.g.  "2026-04-22 00:20:09"  (UTC+7 local time)
// ==============================================
bool getNtpTimestamp(char* buf, size_t bufLen) {
    time_t now = time(nullptr);
    if (now < 1000000000UL) return false;
    struct tm localTm;
    localtime_r(&now, &localTm);
    strftime(buf, bufLen, "%Y-%m-%d %H:%M:%S", &localTm);
    return true;
}

// ==============================================
//  LoRa helpers  (called ONLY from loraTask / Core 0)
//  Never touch WiFi/MQTT from here — use queue instead.
// ==============================================
static void sendAck() {
    radio.clearDio1Action();
    radio.standby();
    int16_t st = radio.transmit((uint8_t*)ACK_PAYLOAD, strlen(ACK_PAYLOAD));
    if (st == RADIOLIB_ERR_NONE) {
        Serial.println("[ACK] Sent OK");
    } else {
        Serial.printf("[ACK] Failed (err %d)\n", st);
    }
    rxReady = false;
    radio.setDio1Action(onRxDone);
    radio.startReceive();
}

// Encode one device entry as JSON and push to the MQTT queue.
// Returns true if enqueued, false if skipped (WiFi offline) or queue full.
static bool enqueueDevice(const char* ts, const char* mac, int8_t rssi,
                          uint16_t globalIndex, uint16_t totalDevices) {
    if (WiFi.status() != WL_CONNECTED) {
        Serial.printf("[Queue] WiFi offline - skip %s\n", mac);
        return false;
    }
    // Normalise MAC → uppercase, no separators (e.g. "76E1C36D275C")
    char macNorm[13] = {0};
    int j = 0;
    for (int i = 0; mac[i] && j < 12; i++) {
        if (mac[i] != ':' && mac[i] != '-') {
            macNorm[j++] = (char)toupper((unsigned char)mac[i]);
        }
    }
    char noStr[12];
    snprintf(noStr, sizeof(noStr), "%u/%u", globalIndex, totalDevices);
    MqttItem item;
    StaticJsonDocument<256> pub;
    pub["ts"]   = ts;
    pub["mac"]  = macNorm;
    pub["rssi"] = rssi;
    pub["no"]   = noStr;
    item.len = (uint16_t)serializeJson(pub, item.payload, sizeof(item.payload));
    if (xQueueSend(xMqttQueue, &item, 0) != pdTRUE) {
        Serial.println("[Queue] FULL - device dropped");
        return false;
    }
    return true;
}

void handlePacket() {
    uint8_t rawBuf[255];
    int16_t st = radio.readData(rawBuf, sizeof(rawBuf));
    size_t  rawLen = radio.getPacketLength();

    if (st == RADIOLIB_ERR_CRC_MISMATCH) {
        Serial.println("[RX] CRC mismatch - discarded");
        radio.startReceive();
        return;
    }
    if (st != RADIOLIB_ERR_NONE) {
        Serial.printf("[RX] Read error (err %d)\n", st);
        radio.startReceive();
        return;
    }

    float gwRssi = radio.getRSSI();
    float gwSnr  = radio.getSNR();
    rxCount++;

    // Send ACK immediately before any processing
    sendAck();

    Serial.printf("[RX] %u bytes | RSSI %.1f dBm | SNR %.1f dB\n",
                  (unsigned)rawLen, gwRssi, gwSnr);

    char ts[24];
    if (!getNtpTimestamp(ts, sizeof(ts))) strncpy(ts, "unknown", sizeof(ts));

    // ───────────────────────────────────────────────
    // Binary packet (magic byte 0xBE)
    // ───────────────────────────────────────────────
    if (rawLen >= (size_t)BIN_HEADER_LEN && rawBuf[0] == BIN_MAGIC) {
        uint8_t  pktIdx      = rawBuf[1];
        uint8_t  totalPkt    = rawBuf[2];
        uint8_t  idLen       = rawBuf[3]; if (idLen > 3) idLen = 3;
        char     nodeId[4]   = {0};
        memcpy(nodeId, &rawBuf[4], idLen);
        uint8_t  cnt         = rawBuf[7];
        uint16_t totalDevices = (uint16_t)rawBuf[8] | ((uint16_t)rawBuf[9] << 8);

        strncpy(sNode, nodeId, sizeof(sNode) - 1);
        sNode[sizeof(sNode) - 1] = '\0';
        snprintf(sRSSI, sizeof(sRSSI), "%.0fdBm", gwRssi);
        snprintf(sLoRa, sizeof(sLoRa), "RX %d/%d", pktIdx + 1, totalPkt);
        oledUpdate();

        Serial.printf("[RX] Binary pkt %d/%d | node=%s | %d devs | total=%u\n",
                      pktIdx + 1, totalPkt, nodeId, cnt, totalDevices);

        if (cnt > 0 && rawLen >= (size_t)(BIN_HEADER_LEN + cnt * BYTES_PER_DEV)) {
            int queued = 0;
            for (uint8_t i = 0; i < cnt; i++) {
                const uint8_t* e = &rawBuf[BIN_HEADER_LEN + i * BYTES_PER_DEV];
                char mac[13];
                snprintf(mac, sizeof(mac), "%02X%02X%02X%02X%02X%02X",
                         e[0], e[1], e[2], e[3], e[4], e[5]);
                uint16_t globalIndex = (uint16_t)pktIdx * DEVS_PER_PKT + i + 1;
                if (enqueueDevice(ts, mac, (int8_t)e[6], globalIndex, totalDevices)) queued++;
            }
            if (queued > 0) {
                snprintf(sMQTT, sizeof(sMQTT), "Q+%d", queued);
            } else {
                strncpy(sMQTT, "NoWiFi", sizeof(sMQTT));
            }
        } else {
            strncpy(sMQTT, "0 devices", sizeof(sMQTT));
        }

        strncpy(sLoRa, "Listening", sizeof(sLoRa));
        oledUpdate();
        return;
    }

    // ───────────────────────────────────────────────
    // Fallback: legacy JSON packet
    // ───────────────────────────────────────────────
    rawBuf[rawLen < sizeof(rawBuf) ? rawLen : sizeof(rawBuf) - 1] = '\0';
    const char* json = (const char*)rawBuf;
    Serial.printf("[RX] JSON payload: %s\n", json);

    strncpy(sLoRa, "Parsing...", sizeof(sLoRa));
    snprintf(sRSSI, sizeof(sRSSI), "%.0fdBm", gwRssi);
    oledUpdate();

    StaticJsonDocument<512> rxDoc;
    DeserializationError err = deserializeJson(rxDoc, json);
    if (err) {
        Serial.printf("[RX] JSON error: %s\n", err.c_str());
        strncpy(sLoRa, "JSON Err", sizeof(sLoRa));
        oledUpdate();
        return;
    }

    const char* nodeId = rxDoc["n"] | "???";
    strncpy(sNode, nodeId, sizeof(sNode) - 1);
    sNode[sizeof(sNode) - 1] = '\0';

    int cnt = 0, queued = 0;
    for (JsonObjectConst dev : rxDoc["d"].as<JsonArrayConst>()) {
        if (enqueueDevice(ts, dev["m"] | "", dev["r"] | 0, 0, 0)) queued++;
        cnt++;
    }
    if (queued > 0) {
        snprintf(sMQTT, sizeof(sMQTT), "Q+%d", queued);
    } else {
        strncpy(sMQTT, cnt > 0 ? "NoWiFi" : "0 devices", sizeof(sMQTT));
    }

    strncpy(sLoRa, "Listening", sizeof(sLoRa));
    oledUpdate();
}

// ==============================================
//  LoRa Task  — Core 0, priority 3
//  Owns: SPI bus, radio object, rxReady flag
// ==============================================
static void loraTask(void* /*pvParameters*/) {
    SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI, SX_NSS);
    int16_t st = radio.begin(LORA_FREQUENCY, LORA_BW, LORA_SF, LORA_CR,
                              RADIOLIB_SX126X_SYNC_WORD_PRIVATE,
                              LORA_TX_POWER, LORA_PREAMBLE);
    if (st != RADIOLIB_ERR_NONE) {
        snprintf(sLoRa, sizeof(sLoRa), "ERR %d", st);
        oledUpdate();
        Serial.printf("[LoRa] Init failed (err %d)\n", st);
        vTaskDelete(nullptr);
        return;
    }
    radio.setDio2AsRfSwitch(true);
    radio.setDio1Action(onRxDone);
    radio.startReceive();
    strncpy(sLoRa, "Listening", sizeof(sLoRa));
    Serial.println("[LoRa] Listening");
    oledUpdate();

    for (;;) {
        if (rxReady) {
            rxReady = false;
            handlePacket();
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

// ==============================================
//  WiFi / MQTT Task  — Core 1, priority 2
//  Owns: WiFiManager, PubSubClient, NTP
//  WiFiManager.autoConnect() is blocking — runs at task start only.
//  After that, reconnects happen non-blocking with vTaskDelay().
// ==============================================
static void wifiMqttTask(void* /*pvParameters*/) {
    // ── Initial WiFi connect ──
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    snprintf(sWiFi, sizeof(sWiFi), "Connecting...");
    oledUpdate();
    Serial.print("[WiFi] Connecting");
    unsigned long t0 = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - t0 < WIFI_TIMEOUT_MS) {
        vTaskDelay(pdMS_TO_TICKS(500));
        Serial.print(".");
    }
    Serial.println();

    if (WiFi.status() == WL_CONNECTED) {
        String ip = WiFi.localIP().toString();
        snprintf(sWiFi, sizeof(sWiFi), "IP:%s", ip.c_str());
        Serial.printf("[WiFi] Connected - %s\n", ip.c_str());
        strncpy(sMQTT, "---", sizeof(sMQTT));
        oledUpdate();

        configTime(NTP_TZ_OFFSET_S, NTP_DST_S, NTP_SERVER1, NTP_SERVER2);
        Serial.print("[NTP] Syncing");
        t0 = millis();
        while (time(nullptr) < 1000000000UL && millis() - t0 < 10000) {
            vTaskDelay(pdMS_TO_TICKS(500));
            Serial.print(".");
        }
        Serial.println(time(nullptr) >= 1000000000UL ? " OK" : " TIMEOUT");
    } else {
        strncpy(sWiFi, "WiFi FAILED", sizeof(sWiFi));
        Serial.println("[WiFi] Connect failed");
        oledUpdate();
    }

    // ── Main loop ──
    for (;;) {
        // 1. Reconnect WiFi if lost (non-blocking with vTaskDelay — LoRa keeps running)
        if (WiFi.status() != WL_CONNECTED) {
            strncpy(sWiFi, "WiFi reconnecting", sizeof(sWiFi));
            strncpy(sMQTT, "---", sizeof(sMQTT));
            oledUpdate();
            WiFi.reconnect();
            unsigned long t0 = millis();
            while (WiFi.status() != WL_CONNECTED && millis() - t0 < WIFI_TIMEOUT_MS) {
                vTaskDelay(pdMS_TO_TICKS(500));
            }
            if (WiFi.status() == WL_CONNECTED) {
                String ip = WiFi.localIP().toString();
                snprintf(sWiFi, sizeof(sWiFi), "IP:%s", ip.c_str());
                Serial.printf("[WiFi] Reconnected - %s\n", ip.c_str());
                configTime(NTP_TZ_OFFSET_S, NTP_DST_S, NTP_SERVER1, NTP_SERVER2);
                oledUpdate();
            } else {
                strncpy(sWiFi, "WiFi lost", sizeof(sWiFi));
                oledUpdate();
                vTaskDelay(pdMS_TO_TICKS(5000));  // back-off before next attempt
                continue;
            }
        }

        // 2. Reconnect MQTT if disconnected
        if (!mqtt.connected()) {
            Serial.printf("[MQTT] Connecting to %s:%d ...", MQTT_BROKER, MQTT_PORT);
            const char* user = strlen(MQTT_USER) ? MQTT_USER : nullptr;
            const char* pass = strlen(MQTT_PASS) ? MQTT_PASS : nullptr;
            bool ok = mqtt.connect(MQTT_CLIENT_ID, user, pass);
            Serial.println(ok ? " OK" : " FAILED");
            if (!ok) {
                strncpy(sMQTT, "NoBroker", sizeof(sMQTT));
                oledUpdate();
                vTaskDelay(pdMS_TO_TICKS(MQTT_RETRY_DELAY_MS));
                continue;
            }
        }

        mqtt.loop();

        // 3. Drain the MQTT queue — publish every queued device payload
        MqttItem item;
        int published = 0;
        while (xQueueReceive(xMqttQueue, &item, 0) == pdTRUE) {
            bool sent = false;
            for (int attempt = 0; attempt < MQTT_MAX_RETRIES && !sent; attempt++) {
                if (!mqtt.connected()) {
                    mqtt.connect(MQTT_CLIENT_ID,
                                 strlen(MQTT_USER) ? MQTT_USER : nullptr,
                                 strlen(MQTT_PASS) ? MQTT_PASS : nullptr);
                    if (!mqtt.connected()) {
                        vTaskDelay(pdMS_TO_TICKS(MQTT_RETRY_DELAY_MS));
                        continue;
                    }
                }
                mqtt.loop();
                sent = mqtt.publish(MQTT_TOPIC, item.payload, (unsigned int)item.len);
                if (!sent) vTaskDelay(pdMS_TO_TICKS(MQTT_RETRY_DELAY_MS));
            }
            if (sent) {
                published++;
                Serial.printf("[MQTT] %s\n", item.payload);
            } else {
                Serial.println("[MQTT] FAILED - dropped");
            }
        }

        if (published > 0) {
            snprintf(sMQTT, sizeof(sMQTT), "OK +%d", published);
            oledUpdate();
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

// ==============================================
//  Initialisation helpers
// ==============================================
static void initOLED() {
    initOledPowerAndReset();
    display.begin();
    display.setContrast(255);
    display.clearBuffer();
    display.setFont(u8g2_font_6x10_tf);
    display.drawStr(0, 10, "=== BLE Receiver ===");
    display.drawStr(0, 30, "Booting...");
    display.sendBuffer();
    delay(800);
}

// ==============================================
//  Arduino entry points
// ==============================================
void setup() {
    Serial.begin(115200);
    delay(100);
    Serial.println("\n=== LoRa P2P - BLE Receiver (RTOS) ===");

    initOLED();  // direct draw — tasks not started yet, no mutex needed

    mqtt.setServer(MQTT_BROKER, MQTT_PORT);
    mqtt.setKeepAlive(60);

    // Create RTOS primitives before tasks start
    xMqttQueue    = xQueueCreate(MQTT_QUEUE_SIZE, sizeof(MqttItem));
    xDisplayMutex = xSemaphoreCreateMutex();

    // LoRa Task : Core 0, prio 3  — receive → ACK → parse → enqueue
    xTaskCreatePinnedToCore(loraTask,     "LoRaTask", 6144,  nullptr, 3, nullptr, 0);
    // WiFi Task : Core 1, prio 2  — WiFiManager → NTP → MQTT → publish
    xTaskCreatePinnedToCore(wifiMqttTask, "WiFiTask", 10240, nullptr, 2, nullptr, 1);
}

void loop() {
    delay(1000);
}
