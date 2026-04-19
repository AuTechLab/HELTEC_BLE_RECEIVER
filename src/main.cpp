/**
 * BLE_Reciver  Heltec WiFi LoRa 32 V3
 *
 * Flow on each LoRa packet arrival:
 *   1. Read packet (interrupt-driven via DIO1)
 *   2. Send ACK back IMMEDIATELY (P2P link-health for sender)
 *   3. Parse JSON payload from sender
 *   4. Get NTP timestamp (ISO-8601 UTC)
 *   5. Publish one MQTT message per BLE device:
 *        topic : MQTT_TOPIC
 *        payload: {"ts":"...","mac":"xx:xx:xx:xx:xx:xx","rssi":-70,
 *                  "node":"S01","gw_rssi":-55.0,"gw_snr":9.2}
 *   6. Update OLED with live status
 *
 * OLED layout (SSD1306 128x64):
 *   == BLE Receiver ==
 *   LoRa: Listening #3
 *   Node: S01 -55dBm
 *   IP  : 192.168.x.x
 *   MQTT: OK 3/3pub
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
#include <WiFiManager.h>
#include <PubSubClient.h>
#include <time.h>

// ==============================================
//  User Configuration  <- Edit these
// ==============================================
// --- WiFiManager: credentials saved to flash (no hardcoding needed) ---
#define WIFI_AP_NAME     "BLE-Receiver"   // AP ที่เปิดตอนยังไม่มี credentials
#define WIFI_AP_PASS     "12345678"        // รหัส AP config portal (min 8 ตัว)
#define CONFIG_BTN_PIN   0                 // BOOT button: กด ค้าง 3 วิตอนเปิด = ล้าง WiFi

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
#define OLED_RST 21
#define OLED_SCL 18
#define OLED_SDA 17

// --- WiFi / MQTT retry ---
#define WIFI_TIMEOUT_MS      15000
#define MQTT_MAX_RETRIES     3
#define MQTT_RETRY_DELAY_MS  2000

// --- ACK sent back to sender ---
static const char ACK_PAYLOAD[] = "{\"ack\":1,\"gw\":\"R01\"}";

// --- Binary packet format (must match sender) ---
#define BIN_MAGIC      0xBE
#define BIN_HEADER_LEN 8
#define BYTES_PER_DEV  7    // 6 MAC + 1 RSSI

// ==============================================
//  Hardware objects
// ==============================================
SX1262    radio   = new Module(SX_NSS, SX_DIO1, SX_RST, SX_BUSY);
U8G2_SSD1306_128X64_NONAME_F_HW_I2C display(U8G2_R0, OLED_RST, OLED_SCL, OLED_SDA);
WiFiClient   wifiClient;
PubSubClient mqtt(wifiClient);
WiFiManager  wifiManager;

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
}

// ==============================================
//  NTP - get ISO-8601 UTC timestamp
//  e.g.  "2026-04-18T10:30:00Z"
// ==============================================
bool getNtpTimestamp(char* buf, size_t bufLen) {
    time_t now = time(nullptr);
    if (now < 1000000000UL) return false;
    struct tm utcTm;
    gmtime_r(&now, &utcTm);
    strftime(buf, bufLen, "%Y-%m-%dT%H:%M:%SZ", &utcTm);
    return true;
}

// ==============================================
//  WiFi (via WiFiManager)
//  ครั้งแรก / หลังล้าง : เปิด AP "BLE-Receiver" → เชื่อม → เปิด 192.168.4.1
//                        → กรอก SSID+Password → บันทึกลง flash
//  บูตปกติ             : auto-connect ด้วย credentials ที่บันทึก
//  ล้าง credentials     : กด BOOT (GPIO0) ค้าง ≥ 3 วิ ตอนเปิดเครื่อง
// ==============================================
static void onConfigPortalStart(WiFiManager*) {
    snprintf(sWiFi, sizeof(sWiFi), "AP:%s", WIFI_AP_NAME);
    strncpy(sMQTT, "Open 192.168.4.1", sizeof(sMQTT));
    oledUpdate();
    Serial.println("[WiFi] Config portal open - connect to AP '" WIFI_AP_NAME "'");
}

bool connectWiFi() {
    if (WiFi.status() == WL_CONNECTED) return true;

    strncpy(sWiFi, "WiFi reconnecting", sizeof(sWiFi));
    oledUpdate();
    WiFi.reconnect();

    unsigned long t0 = millis();
    while (WiFi.status() != WL_CONNECTED) {
        if (millis() - t0 > WIFI_TIMEOUT_MS) {
            strncpy(sWiFi, "WiFi lost", sizeof(sWiFi));
            oledUpdate();
            Serial.println("[WiFi] Reconnect timeout");
            return false;
        }
        delay(500);
    }
    String ip = WiFi.localIP().toString();
    snprintf(sWiFi, sizeof(sWiFi), "IP:%s", ip.c_str());
    Serial.printf("[WiFi] Reconnected - %s\n", ip.c_str());
    oledUpdate();
    return true;
}

// ==============================================
//  MQTT - connect / reconnect
// ==============================================
bool connectMQTT() {
    if (mqtt.connected()) return true;
    if (!connectWiFi()) return false;

    Serial.printf("[MQTT] Connecting to %s:%d ... ", MQTT_BROKER, MQTT_PORT);
    const char* user = strlen(MQTT_USER) ? MQTT_USER : nullptr;
    const char* pass = strlen(MQTT_PASS) ? MQTT_PASS : nullptr;

    bool ok = mqtt.connect(MQTT_CLIENT_ID, user, pass);
    Serial.println(ok ? "OK" : "FAILED");
    return ok;
}

// ==============================================
//  Publish devices from decoded binary packet
// ==============================================
void publishBinaryDevices(const char* nodeId, uint8_t pktIdx, uint8_t totalPkt,
                          const uint8_t* data, uint8_t cnt,
                          float gwRssi, float gwSnr) {
    char ts[24];
    bool hasTime = getNtpTimestamp(ts, sizeof(ts));
    if (!hasTime) strncpy(ts, "unknown", sizeof(ts));

    int published = 0;
    for (uint8_t i = 0; i < cnt; i++) {
        const uint8_t* entry = data + i * BYTES_PER_DEV;
        // Decode 6-byte binary MAC → string
        char mac[18];
        snprintf(mac, sizeof(mac),
                 "%02x:%02x:%02x:%02x:%02x:%02x",
                 entry[0], entry[1], entry[2],
                 entry[3], entry[4], entry[5]);
        int8_t rssi = (int8_t)entry[6];

        StaticJsonDocument<256> pub;
        pub["ts"]       = ts;
        pub["mac"]      = mac;
        pub["rssi"]     = rssi;
       // pub["node"]     = nodeId;
       // pub["gw_rssi"]  = gwRssi;
       // pub["gw_snr"]   = gwSnr;
       // pub["pkt"]      = pktIdx;    // packet index within burst
       //pub["total_pkt"]= totalPkt;  // total packets in burst

        char payload[256];
        size_t len = serializeJson(pub, payload, sizeof(payload));

        bool sent = false;
        for (int attempt = 1; attempt <= MQTT_MAX_RETRIES && !sent; attempt++) {
            mqtt.loop();
            if (!connectMQTT()) {
                snprintf(sMQTT, sizeof(sMQTT), "NoBroker %d/%d", attempt, MQTT_MAX_RETRIES);
                oledUpdate();
                delay(MQTT_RETRY_DELAY_MS);
                continue;
            }
            sent = mqtt.publish(MQTT_TOPIC, payload, (unsigned int)len);
            if (!sent) delay(MQTT_RETRY_DELAY_MS);
        }
        if (sent) {
            published++;
            Serial.printf("[MQTT] %s\n", payload);
        } else {
            Serial.printf("[MQTT] FAILED mac=%s\n", mac);
        }
    }

    snprintf(sMQTT, sizeof(sMQTT), "p%d/%d %d/%dpub",
             pktIdx + 1, totalPkt, published, cnt);
    Serial.printf("[MQTT] Pkt %d/%d: %d/%d published\n",
                  pktIdx + 1, totalPkt, published, cnt);
    oledUpdate();
}

// ==============================================
//  Initialisation
// ==============================================
void initOLED() {
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

void initLoRa() {
    SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI, SX_NSS);
    int16_t st = radio.begin(LORA_FREQUENCY, LORA_BW, LORA_SF, LORA_CR,
                              RADIOLIB_SX126X_SYNC_WORD_PRIVATE,
                              LORA_TX_POWER, LORA_PREAMBLE);
    if (st != RADIOLIB_ERR_NONE) {
        snprintf(sLoRa, sizeof(sLoRa), "ERR %d", st);
        oledUpdate();
        Serial.printf("[LoRa] Init failed (err %d) - halting\n", st);
        while (true) delay(1000);
    }
    radio.setDio2AsRfSwitch(true);
    radio.setDio1Action(onRxDone);
    radio.startReceive();
    strncpy(sLoRa, "Listening", sizeof(sLoRa));
    Serial.println("[LoRa] Listening");
}

// ==============================================
//  Send ACK back to sender
//  DIO1 interrupt is paused during TX to avoid spurious rxReady.
//  radio.standby() ensures clean state before TX regardless of
//  which mode readData() left the chip in.
// ==============================================
void sendAck() {
    radio.clearDio1Action();
    radio.standby();                              // clean state before TX
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

// ==============================================
//  Process received LoRa packet (JSON or binary)
// ==============================================
void handlePacket() {
    // Read raw bytes (not String) to support binary payload
    uint8_t rawBuf[255];
    size_t  rawLen = 0;
    int16_t st = radio.readData(rawBuf, sizeof(rawBuf));
    rawLen = radio.getPacketLength();

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

    // ───────────────────────────────────────────────
    // Binary packet (magic byte 0xBE)
    // ───────────────────────────────────────────────
    if (rawLen >= (size_t)BIN_HEADER_LEN && rawBuf[0] == BIN_MAGIC) {
        uint8_t pktIdx   = rawBuf[1];
        uint8_t totalPkt = rawBuf[2];
        // bytes 3-6: timestamp (ignored on receiver side)
        uint8_t idLen = rawBuf[3]; if (idLen > 3) idLen = 3;
        char    nodeId[4] = {0};
        memcpy(nodeId, &rawBuf[4], idLen);
        uint8_t cnt      = rawBuf[7];

        strncpy(sNode, nodeId, sizeof(sNode) - 1);
        sNode[sizeof(sNode) - 1] = '\0';
        snprintf(sRSSI, sizeof(sRSSI), "%.0fdBm", gwRssi);
        snprintf(sLoRa, sizeof(sLoRa), "RX %d/%d", pktIdx + 1, totalPkt);
        oledUpdate();

        Serial.printf("[RX] Binary pkt %d/%d | node=%s | %d devs\n",
                      pktIdx + 1, totalPkt, nodeId, cnt);

        if (cnt > 0 && rawLen >= (size_t)(BIN_HEADER_LEN + cnt * BYTES_PER_DEV)) {
            publishBinaryDevices(nodeId, pktIdx, totalPkt,
                                 &rawBuf[BIN_HEADER_LEN], cnt,
                                 gwRssi, gwSnr);
        } else {
            strncpy(sMQTT, "0 devices", sizeof(sMQTT));
            oledUpdate();
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

    JsonArrayConst devices = rxDoc["d"].as<JsonArrayConst>();
    if (devices.size() > 0) {
        // Re-use binary publisher with inline data
        // (legacy path: publish directly)
        char ts[24];
        bool hasTime = getNtpTimestamp(ts, sizeof(ts));
        if (!hasTime) strncpy(ts, "unknown", sizeof(ts));
        int published = 0, total = (int)devices.size();
        for (JsonObjectConst dev : devices) {
            StaticJsonDocument<256> pub;
            pub["ts"]      = ts;
            pub["mac"]     = dev["m"] | "";
            pub["rssi"]    = dev["r"] | 0;
            pub["node"]    = nodeId;
            pub["gw_rssi"] = gwRssi;
            pub["gw_snr"]  = gwSnr;
            char payload[256];
            size_t len = serializeJson(pub, payload, sizeof(payload));
            bool sent = false;
            for (int attempt = 1; attempt <= MQTT_MAX_RETRIES && !sent; attempt++) {
                mqtt.loop();
                if (connectMQTT())
                    sent = mqtt.publish(MQTT_TOPIC, payload, (unsigned int)len);
                if (!sent) delay(MQTT_RETRY_DELAY_MS);
            }
            if (sent) published++;
        }
        snprintf(sMQTT, sizeof(sMQTT), "%s %d/%dpub",
                 published == total ? "OK" : "PART", published, total);
    } else {
        strncpy(sMQTT, "0 devices", sizeof(sMQTT));
    }

    strncpy(sLoRa, "Listening", sizeof(sLoRa));
    oledUpdate();
}

// ==============================================
//  Arduino entry points
// ==============================================
void setup() {
    Serial.begin(115200);
    delay(100);
    Serial.println("\n=== LoRa P2P - BLE Receiver (MQTT) ===");

    initOLED();

    mqtt.setServer(MQTT_BROKER, MQTT_PORT);
    mqtt.setKeepAlive(60);

    pinMode(CONFIG_BTN_PIN, INPUT_PULLUP);

    wifiManager.setConnectTimeout(20);
    wifiManager.setConfigPortalTimeout(180);
    wifiManager.setAPCallback(onConfigPortalStart);

    if (wifiManager.autoConnect(WIFI_AP_NAME, WIFI_AP_PASS)) {
        String ip = WiFi.localIP().toString();
        snprintf(sWiFi, sizeof(sWiFi), "IP:%s", ip.c_str());
        Serial.printf("[WiFi] Connected - %s\n", ip.c_str());
        strncpy(sMQTT, "---", sizeof(sMQTT));
        oledUpdate();
        // Sync NTP
        configTime(NTP_TZ_OFFSET_S, NTP_DST_S, NTP_SERVER1, NTP_SERVER2);
        Serial.print("[NTP] Syncing");
        unsigned long t0 = millis();
        while (time(nullptr) < 1000000000UL && millis() - t0 < 10000) {
            delay(500);
            Serial.print(".");
        }
        Serial.println(time(nullptr) >= 1000000000UL ? " OK" : " TIMEOUT");
        connectMQTT();
    } else {
        strncpy(sWiFi, "WiFi FAILED", sizeof(sWiFi));
        Serial.println("[WiFi] Connect/portal failed");
        oledUpdate();
    }

    initLoRa();

    oledUpdate();
}

void loop() {
    if (mqtt.connected()) mqtt.loop();

    // กด BOOT (GPIO0) ค้าง 3 วิ ตอนไหนก็ได้ = ล้าง WiFi credentials
    static unsigned long btnPressStart = 0;
    if (digitalRead(CONFIG_BTN_PIN) == LOW) {
        if (btnPressStart == 0) btnPressStart = millis();
        if (millis() - btnPressStart >= 3000) {
            strncpy(sWiFi, "WiFi reset! Reboot", sizeof(sWiFi));
            strncpy(sMQTT, "---", sizeof(sMQTT));
            oledUpdate();
            Serial.println("[WiFi] Credentials cleared - restarting");
            wifiManager.resetSettings();
            delay(500);
            ESP.restart();
        }
    } else {
        btnPressStart = 0;
    }

    if (rxReady) {
        rxReady = false;
        handlePacket();
    }
}
