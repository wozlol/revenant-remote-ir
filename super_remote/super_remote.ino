// Revenant 2.4, M5StickS3
// Learn and send IR, driven from the web tool over Bluetooth. Settings live in flash.
// Front button: send the last heard code back out (echo test). Right button: menu.
// Power button: double click for full off. Silent by design.

#include <M5Unified.h>
#include "driver/rmt_rx.h"
#include "driver/rmt_tx.h"
#include <LittleFS.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <BLEHIDDevice.h>
#include <BLESecurity.h>
#include <Update.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include "mbedtls/aes.h"
#include <WebServer.h>
#include <ESPmDNS.h>
#include <deque>
#include "driver/gpio.h"
#include "soc/gpio_struct.h"
#include "esp_rom_gpio.h"

#define FW_VERSION "2.7"
#define PROTO_VERSION 5
#define C565(r, g, b) ((((r) & 0xF8) << 8) | (((g) & 0xFC) << 3) | ((b) >> 3))

#define SVC_UUID "7a3f0001-5d4e-4b6a-9c1f-0a0d5e0c0001"
#define RX_UUID "7a3f0002-5d4e-4b6a-9c1f-0a0d5e0c0001"
#define TX_UUID "7a3f0003-5d4e-4b6a-9c1f-0a0d5e0c0001"

static const int IR_RX_PIN = 42;
static const int IR_TX_PIN = 46;
static const int IR_TX2_PIN = 9;    // M5 IR module on the Grove port: yellow wire is its LED
static const int IR_RX2_PIN = 10;   // white wire is its receiver
static const uint32_t MERGE_MS = 8; // after one receiver finishes a frame, wait this long for the other
static const size_t RX_SYMS = 256;
static const uint16_t MAX_T = 512;
static uint16_t minMarkUs = 200;   // shorter marks are backlight noise, real remotes use 400 us or more
static uint16_t tolAbs = 250;      // learning: wiggle room in us for short pulses
static uint8_t tolPct = 15;        // learning: wiggle room in percent for long pulses
static bool prefixOn = true;       // learning: accept a press whose tail was cut off
static uint8_t sendRepeats = 1;     // mapping: send each code at least this many times, real remotes send one frame per tap
static uint8_t txDuty = 33;         // IR carrier duty in percent, 1/3 is the usual standard for NEC and Sony remotes
static uint16_t sendWaitMs = 60;    // mapping: how long the remote must be quiet before the stick sends, so the two never talk over each other
static uint32_t lastRxAnyMs = 0;    // last time the receiver heard anything real
static uint8_t shapeTol = 1;       // learning: symbols that may differ when comparing long presses by shape
static const size_t MAX_LINE = 70000;

static const uint16_t C_TEXT = C565(220, 220, 220);
static const uint16_t C_DIM = C565(110, 110, 110);
static const uint16_t C_LINE = C565(40, 40, 40);
static const uint16_t C_HI = C565(255, 190, 0);
static const uint16_t C_NEW = C565(0, 230, 90);
static const uint16_t C_SAME = C565(255, 150, 0);
static const uint16_t C_BAD = C565(255, 70, 70);
static const uint16_t C_BT = C565(80, 160, 255);

struct Capture {
  uint16_t t[MAX_T];  // microseconds, mark first, then alternating space and mark
  uint16_t n;
  uint32_t total;
  uint32_t seq;
  uint8_t burst;
  bool valid;
};

struct Guess {
  const char* name;
  int bits;
};

static const uint8_t BRIGHT[] = {6, 20, 50, 110, 220};
static const char* const BRIGHT_NAME[] = {"Night", "Dim", "Mid", "High", "Max"};
static const uint16_t GAP_US[] = {6000, 12000, 25000};
static const char* const GAP_NAME[] = {"6 ms", "12 ms", "25 ms"};
static const int MENU_ITEMS = 7;
static uint32_t forgetShownMs = 0;

static M5Canvas cv(&M5.Display);
static Capture cap;
static uint16_t tmpT[MAX_T];
static uint16_t frT[MAX_T];
static uint16_t refT[MAX_T];  // first frame of the previous press
static uint16_t refN = 0;
static uint16_t sendT[MAX_T];

static rmt_symbol_word_t rxBuf[RX_SYMS];
static volatile size_t rxNum = 0;
static volatile bool rxDone = false;
static rmt_channel_handle_t rxChan = nullptr;
static rmt_symbol_word_t rxBuf2[RX_SYMS];
static volatile size_t rxNum2 = 0;
static volatile bool rxDone2 = false;
static rmt_channel_handle_t rxChan2 = nullptr;
static bool rx2Up = false;
static bool tx2Up = false;
static int idleLevel2 = 1;
struct RxCap { uint16_t t[MAX_T]; uint16_t n; uint32_t at; bool has; };
static RxCap rxCap[2];
static uint8_t lastSrc = 0;         // which receiver supplied the last frame, 0 built-in, 1 module
static uint32_t srcWins[2] = {0, 0};
static rmt_channel_handle_t txChan = nullptr;
static rmt_encoder_handle_t copyEnc = nullptr;
static rmt_symbol_word_t txSyms[MAX_T + 8];
static uint32_t txCarrier = 0;
static uint32_t rxIgnoreUntil = 0;

static BLEServer* bleServer = nullptr;
static BLECharacteristic* bleTx = nullptr;
static volatile bool bleConnected = false;   // the web tool is connected and listening
static volatile bool kbConnected = false;    // a host (Apple TV, phone, computer) is paired and listening to the keyboard
static bool kbReported = false;
static uint16_t webHandle = 0xFFFF;
static uint16_t kbHandle = 0xFFFF;
static BLEHIDDevice* hid = nullptr;
static BLECharacteristic* inKey = nullptr;
static BLECharacteristic* inCons = nullptr;
static BLECharacteristic* inMouse = nullptr;
extern "C" int ble_store_clear(void);

// firmware update over Bluetooth: the web tool streams the app image, the stick writes it to the other app slot
static const uint32_t OTA_RING = 32768;
static volatile bool otaActive = false;
static volatile bool otaAbort = false;
static volatile bool otaOverflow = false;
static uint8_t* otaBuf = nullptr;
static volatile uint32_t otaHead = 0;  // bytes received, only ever grows
static volatile uint32_t otaTail = 0;  // bytes written to flash
static uint32_t otaSize = 0;
static uint32_t otaAckAt = 0;
static uint32_t otaDoneMs = 0;
static volatile uint32_t otaLastMs = 0;  // last time firmware bytes arrived
static volatile uint16_t bleMtu = 23;
static String bleAcc;
static std::deque<String> inLines;
static SemaphoreHandle_t inMux;
static String outBuf;
static uint32_t lastNotifyMs = 0;

static int W = 135;
static int H = 240;
static int batt = -1;
static int idleLevel = 1;
static bool rxUp = false;
static bool txUp = false;
static uint8_t brIdx = 1;
static uint8_t gapIdx = 0;
static const uint16_t SLEEP_S[] = {0, 15, 30, 60, 120, 300};
static const char* const SLEEP_NAME[] = {"Never", "15 sec", "30 sec", "1 min", "2 min", "5 min"};
static uint8_t sleepIdx = 2;
static bool bleKbMode = false;     // Bluetooth is either the settings link or a keyboard for TVs, never both at once
static bool bleModeSaved = false;
static uint32_t restartAt = 0;
static bool wifiOn = true;      // Wi-Fi power, kept when the stick restarts
static bool asleep = false;     // screen off, everything else keeps working
static float imuMaxD = 0, imuMaxSpin = 0;   // biggest motion since the last log line, for finding the right wake level
static bool imuSeen = false;
static uint32_t lastActMs = 0;
static void wake();
static void prefsSave();
static bool menuOpen = false;
static uint8_t menuSel = 0;
static bool isNew = true;
static uint32_t newCount = 0;
static uint32_t noiseCount = 0;
static uint32_t lastFrameMs = 0;
static uint32_t lastBattMs = 0;
static uint32_t lastDrawMs = 0;
static bool dirty = true;
static char lastTxName[24] = "";
static uint32_t txCount = 0;
static uint32_t drawCount = 0;
static volatile int stage = 0;
static volatile uint32_t loopCount = 0;

static void beatTask(void*) {
  for (;;) {
    Serial.printf("beat t=%lus stage=%d loops=%lu draws=%lu reset=%d heap=%u\n", (unsigned long)(millis() / 1000), (int)stage, (unsigned long)loopCount, (unsigned long)drawCount, (int)esp_reset_reason(), (unsigned)ESP.getFreeHeap());
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

// ---------- Bluetooth ----------

static String* replySink = nullptr;      // set while a Wi-Fi request is being answered
static const int EV_MAX = 40;
static String evBuf[EV_MAX];             // recent unsolicited lines for the Wi-Fi page to poll
static uint32_t evSeq = 0;
static uint32_t httpLastPoll = 0;
static bool wifiUp = false;
static String wifiIp;
static const char* wifiMsg = "Off";
static bool otaShow = false;   // draw the update screen for a Wi-Fi update
static int otaPct = 0;
static String apiKey;                    // secret the web page must send over Wi-Fi, handed out only over Bluetooth
static uint32_t openUntil = 0;           // Bluetooth commands that change anything wait for a press of the front button
static bool wasOpen = false;
static bool windowOpen() { return otaActive || (int32_t)(openUntil - millis()) > 0; }

static void bleSend(const String& s) {
  if (replySink) {
    *replySink += s;
    *replySink += '\n';
    return;
  }
  if (!bleConnected) return;
  if (outBuf.length() > 90000) return;
  outBuf += s;
  outBuf += '\n';
}

// a line nobody asked for: heard IR, a fired rule, keyboard change. Goes to whoever is listening.
static void emit(const String& s) {
  bleSend(s);
  if (wifiUp && millis() - httpLastPoll < 8000) {
    evBuf[evSeq % EV_MAX] = s;
    evSeq++;
  }
}

static void bleFlush() {
  if (!bleConnected || outBuf.length() == 0) return;
  if (millis() - lastNotifyMs < 7) return;
  size_t chunk = bleMtu > 23 ? bleMtu - 3 : 20;
  if (chunk > 180) chunk = 180;
  if (chunk > outBuf.length()) chunk = outBuf.length();
  bleTx->setValue((uint8_t*)outBuf.c_str(), chunk);
  bleTx->notify();
  outBuf.remove(0, chunk);
  lastNotifyMs = millis();
}

class ServerCb : public BLEServerCallbacks {
  void onConnect(BLEServer* s, ble_gap_conn_desc* d) override {
    Serial.printf("ble connect handle=%d\n", d->conn_handle);
    dirty = true;
    BLEDevice::startAdvertising();  // stay findable so the web tool and the TV can both connect
  }
  void onDisconnect(BLEServer* s, ble_gap_conn_desc* d) override {
    Serial.printf("ble disconnect handle=%d web=%d kb=%d\n", d->conn_handle, webHandle, kbHandle);
    if (d->conn_handle == webHandle) {
      if (otaActive) otaAbort = true;
      bleConnected = false;
      webHandle = 0xFFFF;
      bleMtu = 23;
    }
    if (d->conn_handle == kbHandle) {
      kbConnected = false;
      kbHandle = 0xFFFF;
    }
    BLEDevice::startAdvertising();
    dirty = true;
  }
  void onMtuChanged(BLEServer* s, ble_gap_conn_desc* d, uint16_t mtu) override {
    if (d->conn_handle == webHandle) bleMtu = mtu;
  }
};

// the web tool listens on the TX characteristic, the paired host listens on the keyboard report
class TxSubCb : public BLECharacteristicCallbacks {
  void onSubscribe(BLECharacteristic* c, ble_gap_conn_desc* d, uint16_t sub) override {
    bleConnected = (sub & 1) != 0;
    webHandle = bleConnected ? d->conn_handle : 0xFFFF;
    dirty = true;
  }
};
class KbSubCb : public BLECharacteristicCallbacks {
  void onSubscribe(BLECharacteristic* c, ble_gap_conn_desc* d, uint16_t sub) override {
    Serial.printf("ble keyboard subscribe=%d handle=%d\n", sub, d->conn_handle);
    kbConnected = (sub & 1) != 0;
    kbHandle = kbConnected ? d->conn_handle : 0xFFFF;
    dirty = true;
  }
};
class SecCb : public BLESecurityCallbacks {
  bool onSecurityRequest() { return true; }
  void onAuthenticationComplete(ble_gap_conn_desc* d) {
    Serial.printf("ble auth handle=%d encrypted=%d bonded=%d\n", d->conn_handle, d->sec_state.encrypted, d->sec_state.bonded);
    dirty = true;
  }
  uint32_t onPassKeyRequest() { return 0; }
  void onPassKeyNotify(uint32_t k) {}
  bool onConfirmPIN(uint32_t k) { return true; }
};

class RxCb : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* c, ble_gap_conn_desc* d) override {
    String v = c->getValue();
    if (otaActive) {  // raw firmware bytes, not text lines
      size_t n = v.length();
      if (!otaBuf || n > OTA_RING - (otaHead - otaTail)) { otaOverflow = true; return; }
      for (size_t i = 0; i < n; i++) otaBuf[(otaHead + i) % OTA_RING] = (uint8_t)v[i];
      otaHead = otaHead + n;
      otaLastMs = millis();
      return;
    }
    for (size_t i = 0; i < v.length(); i++) {
      char ch = v[i];
      if (ch == '\n') {
        xSemaphoreTake(inMux, portMAX_DELAY);
        inLines.push_back(bleAcc);
        xSemaphoreGive(inMux);
        bleAcc = "";
      } else if (ch != '\r') {
        if (bleAcc.length() < MAX_LINE) bleAcc += ch;
      }
    }
  }
};

static ServerCb serverCb;
static TxSubCb txSubCb;
class LogSubCb : public BLECharacteristicCallbacks {
  void onSubscribe(BLECharacteristic* c, ble_gap_conn_desc* d, uint16_t sub) override {
    Serial.printf("ble report subscribe=%d handle=%d chr=%d\n", sub, d->conn_handle, c->getHandle());
  }
};
static KbSubCb kbSubCb;
static LogSubCb logSubCb;
static RxCb rxCb;

static const uint8_t HID_MAP[] = {
  // keyboard, report 1: modifiers, reserved, six keys
  0x05, 0x01, 0x09, 0x06, 0xA1, 0x01, 0x85, 0x01,
  0x05, 0x07, 0x19, 0xE0, 0x29, 0xE7, 0x15, 0x00, 0x25, 0x01, 0x75, 0x01, 0x95, 0x08, 0x81, 0x02,
  0x95, 0x01, 0x75, 0x08, 0x81, 0x01,
  0x95, 0x06, 0x75, 0x08, 0x15, 0x00, 0x25, 0x65, 0x05, 0x07, 0x19, 0x00, 0x29, 0x65, 0x81, 0x00,
  0xC0,
  // media keys, report 2: one 16 bit usage
  0x05, 0x0C, 0x09, 0x01, 0xA1, 0x01, 0x85, 0x02,
  0x15, 0x00, 0x26, 0xFF, 0x03, 0x19, 0x00, 0x2A, 0xFF, 0x03, 0x75, 0x10, 0x95, 0x01, 0x81, 0x00,
  0xC0,
  // mouse, report 3: three buttons, x, y, wheel
  0x05, 0x01, 0x09, 0x02, 0xA1, 0x01, 0x85, 0x03, 0x09, 0x01, 0xA1, 0x00,
  0x05, 0x09, 0x19, 0x01, 0x29, 0x03, 0x15, 0x00, 0x25, 0x01, 0x95, 0x03, 0x75, 0x01, 0x81, 0x02,
  0x95, 0x01, 0x75, 0x05, 0x81, 0x01,
  0x05, 0x01, 0x09, 0x30, 0x09, 0x31, 0x09, 0x38, 0x15, 0x81, 0x25, 0x7F, 0x75, 0x08, 0x95, 0x03, 0x81, 0x06,
  0xC0, 0xC0,
};

// The board core (3.3.10) drops a second characteristic that has the same UUID as one the service already has.
// A HID keyboard needs one Input Report characteristic (0x2A4D) per report id, so the media and mouse reports
// never reached the table and the first one sent crashed the link. This puts them in, the way the same fix
// is done in jinwon-int/esp32-iphone-agent #105, by reaching the service's private map.
template <typename Tag, typename Tag::type M> struct SvcRob { friend typename Tag::type svcMapOf(Tag) { return M; } };
struct SvcMapTag { typedef BLECharacteristicMap BLEService::*type; friend type svcMapOf(SvcMapTag); };
template struct SvcRob<SvcMapTag, &BLEService::m_characteristicMap>;

static BLECharacteristic* hidInput(uint8_t id) {
  BLECharacteristic* c = hid->inputReport(id);
  BLECharacteristicMap& m = hid->hidService()->*svcMapOf(SvcMapTag());
  bool have = false;
  for (BLECharacteristic* p = m.getFirst(); p; p = m.getNext()) if (p == c) have = true;
  if (!have) m.setByUUID(c, c->getUUID());
  return c;
}

extern "C" int ble_svc_gap_device_appearance_set(uint16_t appearance);

static void bleStart(bool kb) {
  inMux = xSemaphoreCreateMutex();
  BLEDevice::setMTU(247);
  BLEDevice::init(kb ? "Revenant Keyboard" : "Revenant Setup");
  bleServer = BLEDevice::createServer();
  bleServer->setCallbacks(&serverCb);
  BLEAdvertising* adv = BLEDevice::getAdvertising();
  BLEAdvertisementData ad, sr;
  ad.setFlags(0x06);

  if (kb) {
    // keyboard mode: looks like a keyboard to a TV, and offers nothing else
    BLESecurity* sec = new BLESecurity();
    sec->setCapability(ESP_IO_CAP_NONE);  // Just Works pairing, no code to type
    sec->setAuthenticationMode(true, false, true);
    BLESecurity::setInitEncryptionKey(ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK);  // hand the host both the encryption and the identity key, as the working keyboard libraries do
    BLESecurity::setRespEncryptionKey(ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK);
    BLESecurity::setForceAuthentication(true);  // ask the host to encrypt right after it connects, like Espressif's HID demo, so the TV never has to guess
    BLEDevice::setSecurityCallbacks(new SecCb());
    hid = new BLEHIDDevice(bleServer);
    hid->manufacturer()->setValue("Revenant");
    hid->pnp(0x02, 0xe502, 0xa111, 0x0210);  // neutral ids, never borrow Apple's
    hid->hidInfo(0x00, 0x01);
    hid->reportMap((uint8_t*)HID_MAP, sizeof(HID_MAP));
    inKey = hidInput(1);
    inCons = hidInput(2);
    inMouse = hidInput(3);
    inKey->setCallbacks(&kbSubCb);
    inCons->setCallbacks(&logSubCb);
    inMouse->setCallbacks(&logSubCb);
    hid->setBatteryLevel(100);
    hid->startServices();
    // advertise the way the proven ESP32 keyboard libraries do: appearance and the HID service, name follows from the device name
    adv->setAppearance(0x03C1);
    ble_svc_gap_device_appearance_set(0x03C1);
    adv->addServiceUUID(hid->hidService()->getUUID());
    adv->addServiceUUID(hid->batteryService()->getUUID());
    adv->setMinInterval(0x20);  // 20 to 40 ms, quick to find and connect, what the maintained HID libraries use
    adv->setMaxInterval(0x40);
    adv->setScanResponse(true);
    BLEDevice::startAdvertising();
    return;
  } else {
    // setup mode: only the settings link, no keyboard, so computers never pair it as one
    BLEService* svc = bleServer->createService(SVC_UUID);
    BLECharacteristic* rx = svc->createCharacteristic(RX_UUID, BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
    rx->setCallbacks(&rxCb);
    bleTx = svc->createCharacteristic(TX_UUID, BLECharacteristic::PROPERTY_NOTIFY);
    bleTx->addDescriptor(new BLE2902());
    bleTx->setCallbacks(&txSubCb);
    svc->start();
    ad.setCompleteServices(BLEUUID(SVC_UUID));
    sr.setName("Revenant Setup");
  }
  adv->setAdvertisementData(ad);
  adv->setScanResponseData(sr);
  adv->setScanResponse(true);
  BLEDevice::startAdvertising();
}

// ---------- Bluetooth keyboard, media keys, mouse ----------

static bool kbNotify(BLECharacteristic* c, const uint8_t* d, size_t n) {
  if (!kbConnected || !c) { Serial.printf("kb notify skipped connected=%d chr=%d\n", (int)kbConnected, c ? (int)c->getHandle() : -1); return false; }
  c->setValue((uint8_t*)d, n);
  c->notify();
  Serial.printf("kb notify chr=%d n=%u first=%02x second=%02x third=%02x\n", (int)c->getHandle(), (unsigned)n, n > 0 ? d[0] : 0, n > 1 ? d[1] : 0, n > 2 ? d[2] : 0);
  return true;
}

static bool kbTap(uint8_t mod, uint8_t code) {
  uint8_t r[8] = {mod, 0, code, 0, 0, 0, 0, 0};
  if (!kbNotify(inKey, r, 8)) return false;
  delay(15);
  memset(r, 0, 8);
  kbNotify(inKey, r, 8);
  return true;
}

static bool consTap(uint16_t usage) {
  uint8_t r[2] = {(uint8_t)(usage & 0xFF), (uint8_t)(usage >> 8)};
  if (!kbNotify(inCons, r, 2)) return false;
  delay(15);
  r[0] = r[1] = 0;
  kbNotify(inCons, r, 2);
  return true;
}

static bool mouseSend(uint8_t btn, int8_t dx, int8_t dy, int8_t wheel) {
  uint8_t r[4] = {btn, (uint8_t)dx, (uint8_t)dy, (uint8_t)wheel};
  if (!kbNotify(inMouse, r, 4)) return false;
  if (btn) {
    delay(15);
    r[0] = 0; r[1] = 0; r[2] = 0; r[3] = 0;
    kbNotify(inMouse, r, 4);
  }
  return true;
}

// ---------- IR receive ----------

static bool IRAM_ATTR onRecvDone(rmt_channel_handle_t ch, const rmt_rx_done_event_data_t* e, void* user) {
  rxNum = e->num_symbols;
  rxDone = true;
  return false;
}

static bool IRAM_ATTR onRecvDone2(rmt_channel_handle_t ch, const rmt_rx_done_event_data_t* e, void* user) {
  rxNum2 = e->num_symbols;
  rxDone2 = true;
  return false;
}

static bool rxInit() {
  rmt_rx_channel_config_t c = {};
  c.gpio_num = (gpio_num_t)IR_RX_PIN;
  c.clk_src = RMT_CLK_SRC_DEFAULT;
  c.resolution_hz = 1000000;
  c.mem_block_symbols = 128;
  if (rmt_new_rx_channel(&c, &rxChan) != ESP_OK) return false;
  rmt_rx_event_callbacks_t cb = {};
  cb.on_recv_done = onRecvDone;
  if (rmt_rx_register_event_callbacks(rxChan, &cb, nullptr) != ESP_OK) return false;
  return rmt_enable(rxChan) == ESP_OK;
}

// The M5 IR module's receiver, if it is plugged in. Its output idles high like the built-in one.
static bool rx2Init() {
  gpio_set_pull_mode((gpio_num_t)IR_RX2_PIN, GPIO_PULLUP_ONLY);
  rmt_rx_channel_config_t c = {};
  c.gpio_num = (gpio_num_t)IR_RX2_PIN;
  c.clk_src = RMT_CLK_SRC_DEFAULT;
  c.resolution_hz = 1000000;
  c.mem_block_symbols = 48;  // the one block left over, longer frames stream through it
  if (rmt_new_rx_channel(&c, &rxChan2) != ESP_OK) return false;
  rmt_rx_event_callbacks_t cb = {};
  cb.on_recv_done = onRecvDone2;
  if (rmt_rx_register_event_callbacks(rxChan2, &cb, nullptr) != ESP_OK) return false;
  return rmt_enable(rxChan2) == ESP_OK;
}

static void rxArm() {
  rmt_receive_config_t rc = {};
  rc.signal_range_min_ns = 1000;
  rc.signal_range_max_ns = (uint32_t)GAP_US[gapIdx] * 1000;
  rxDone = false;
  rmt_receive(rxChan, rxBuf, sizeof(rxBuf), &rc);
}

static void rx2Arm() {
  rmt_receive_config_t rc = {};
  rc.signal_range_min_ns = 1000;
  rc.signal_range_max_ns = (uint32_t)GAP_US[gapIdx] * 1000;
  rxDone2 = false;
  rmt_receive(rxChan2, rxBuf2, sizeof(rxBuf2), &rc);
}

static bool nearT(uint16_t v, uint16_t target) {
  int d = (int)v - (int)target;
  if (d < 0) d = -d;
  return d <= (int)target * 25 / 100;
}

static Guess guessProtocol(const Capture& c) {
  Guess g = {"Unknown", -1};
  if (c.n < 3) return g;
  uint16_t m = c.t[0];
  uint16_t s = c.t[1];
  if (nearT(m, 9000) && nearT(s, 4500)) {
    g.name = "NEC";
    g.bits = (c.n - 3) / 2;
  } else if (nearT(m, 9000) && nearT(s, 2250)) {
    g.name = "NEC rpt";
  } else if (nearT(m, 4500) && nearT(s, 4500)) {
    g.name = "Samsung";
    g.bits = (c.n - 3) / 2;
  } else if (nearT(m, 3456) && nearT(s, 1728)) {
    g.name = "Panasonic";
    g.bits = (c.n - 3) / 2;
  } else if (nearT(m, 2400) && nearT(s, 600)) {
    g.name = "Sony";
    g.bits = (c.n - 1) / 2;
  } else if (nearT(m, 8400) && nearT(s, 4200)) {
    g.name = "JVC";
    g.bits = (c.n - 3) / 2;
  }
  return g;
}

// how far a heard time may differ from a stored one: short pulses wobble most in absolute terms, long ones in proportion
static bool sameCode(const uint16_t* a, const uint16_t* b, uint16_t n) {
  for (uint16_t i = 0; i < n; i++) {
    int d = (int)a[i] - (int)b[i];
    if (d < 0) d = -d;
    int lim = (int)b[i] * tolPct / 100;
    if (lim < tolAbs) lim = tolAbs;
    if (d > lim) return false;
  }
  return true;
}

// the shape of a frame: every time becomes short, medium or long. Old remotes jitter in microseconds but not in shape.
static inline uint8_t shapeCls(uint16_t v) { return v < 1000 ? 0 : v < 4500 ? 1 : 2; }

// how many symbols differ between two frames by shape, 99 when they are too short or too long to judge this way
static uint8_t shapeDist(const uint16_t* a, uint16_t an, const uint16_t* b, uint16_t bn) {
  if (an < 20 || bn < 20 || an > 127 || bn > 127) return 99;
  uint8_t prev[128], cur[128];
  for (uint16_t j = 0; j <= bn; j++) prev[j] = (uint8_t)j;
  for (uint16_t i = 1; i <= an; i++) {
    cur[0] = (uint8_t)i;
    uint8_t ca = shapeCls(a[i - 1]);
    for (uint16_t j = 1; j <= bn; j++) {
      uint8_t sub = prev[j - 1] + (ca == shapeCls(b[j - 1]) ? 0 : 1);
      uint8_t del = prev[j] + 1;
      uint8_t ins = cur[j - 1] + 1;
      uint8_t m = sub < del ? sub : del;
      cur[j] = m < ins ? m : ins;
    }
    memcpy(prev, cur, bn + 1);
  }
  return prev[bn];
}

// heard frame against a stored frame: 1 same, 2 a clean cut off start of it (a dropped tail), 0 different
static int frameMatch(const uint16_t* h, uint16_t hn, const uint16_t* r, uint16_t rn) {
  if (hn == rn || (hn > rn && hn <= rn + 2)) return sameCode(h, r, rn) ? 1 : 0;
  if (prefixOn && hn < rn && hn >= 11 && hn * 2 >= rn) return sameCode(h, r, hn - 1) ? 2 : 0;
  return 0;
}

// ---------- mapping: a heard remote button fires a list of outputs, all on the stick ----------

struct Act {
  char type;  // I infrared, K keyboard key, C media key, M mouse
  uint32_t hz;
  uint8_t rep;
  uint16_t gap;
  uint16_t n;
  uint16_t* t;
  int a, b, c, d;
};
struct Rule {
  char name[24];
  bool hold;  // keep firing while the remote button is held
  uint32_t lastFire;
  uint16_t n;
  uint16_t* t;
  std::vector<Act> acts;
};
static std::vector<Rule> rules;
static volatile int mapPending = -1;
static uint32_t mapPendingSince = 0;
static int mapLastRule = -1;
static uint32_t mapLastMs = 0;

static void rulesClear() {
  for (auto& r : rules) {
    free(r.t);
    for (auto& a : r.acts) free(a.t);
  }
  rules.clear();
  mapPending = -1;
  mapLastRule = -1;
}

static uint16_t parseTimes(const char* p, uint16_t* out, uint16_t max) {
  uint16_t n = 0;
  char* e;
  while (n < max) {
    while (*p == ' ') p++;
    if (!*p) break;
    out[n] = (uint16_t)strtoul(p, &e, 10);
    if (e == p) break;
    n++;
    p = e;
  }
  return n;
}

static uint16_t* dupTimes(const char* p, uint16_t* nOut) {
  static uint16_t tmp[MAX_T];
  uint16_t n = parseTimes(p, tmp, MAX_T);
  *nOut = n;
  if (!n) return nullptr;
  uint16_t* m = (uint16_t*)malloc(n * sizeof(uint16_t));
  if (m) memcpy(m, tmp, n * sizeof(uint16_t));
  return m;
}

// one rule per line: R|name|hold|source timings|action|action
static bool ruleParse(const String& line) {
  if (!line.startsWith("R|")) return false;
  std::vector<String> f;
  int from = 2;
  for (;;) {
    int k = line.indexOf('|', from);
    if (k < 0) { f.push_back(line.substring(from)); break; }
    f.push_back(line.substring(from, k));
    from = k + 1;
  }
  if (f.size() < 4 || rules.size() >= 64) return false;
  Rule r;
  memset(r.name, 0, sizeof r.name);
  strncpy(r.name, f[0].c_str(), 22);
  r.hold = f[1] == "1";
  r.lastFire = 0;
  r.t = dupTimes(f[2].c_str(), &r.n);
  if (!r.t || r.n < 3) { free(r.t); return false; }
  for (size_t i = 3; i < f.size() && r.acts.size() < 8; i++) {
    Act a = {};
    const char* p = f[i].c_str();
    a.type = *p;
    p += 1;
    char* e;
    if (a.type == 'I') {
      a.hz = strtoul(p, &e, 10); p = e;
      a.rep = (uint8_t)strtoul(p, &e, 10); p = e;
      a.gap = (uint16_t)strtoul(p, &e, 10); p = e;
      a.t = dupTimes(p, &a.n);
      if (!a.t || a.n < 2 || a.hz < 20000 || a.hz > 60000 || a.rep < 1 || a.rep > 20) { free(a.t); continue; }
    } else if (a.type == 'K') {
      a.a = (int)strtol(p, &e, 10); p = e;
      a.b = (int)strtol(p, &e, 10);
    } else if (a.type == 'C') {
      a.a = (int)strtol(p, &e, 10);
    } else if (a.type == 'M') {
      a.a = (int)strtol(p, &e, 10); p = e;
      a.b = (int)strtol(p, &e, 10); p = e;
      a.c = (int)strtol(p, &e, 10); p = e;
      a.d = (int)strtol(p, &e, 10);
    } else {
      continue;
    }
    r.acts.push_back(a);
  }
  if (r.acts.empty()) { free(r.t); return false; }
  rules.push_back(r);
  return true;
}

static void rulesLoad() {
  rulesClear();
  File f = LittleFS.open("/map.txt", "r");
  if (!f) return;
  while (f.available()) {
    String l = f.readStringUntil('\n');
    l.trim();
    if (l.length()) ruleParse(l);
  }
  f.close();
}

static bool mapFiredThisPress = false;

static void mapOnFrame(const uint16_t* t, uint16_t n, bool first) {
  if (rules.empty()) return;
  uint32_t now = millis();
  if (first) mapFiredThisPress = false;
  int full = -1, pref = -1, prefCount = 0;
  for (size_t i = 0; i < rules.size(); i++) {
    int m = frameMatch(t, n, rules[i].t, rules[i].n);
    if (m == 1 && full < 0) full = (int)i;
    else if (m == 2) { prefCount++; pref = (int)i; }
  }
  // a cut off frame only counts when exactly one rule fits it
  int hit = full >= 0 ? full : (prefCount == 1 ? pref : -1);
  if (hit < 0 && prefCount == 0) {  // last resort for jittery remotes: closest by shape, only when it clearly beats the runner up
    int bestI = -1;
    uint8_t best = 99, second = 99;
    for (size_t i = 0; i < rules.size(); i++) {
      uint8_t d = shapeDist(t, n, rules[i].t, rules[i].n);
      if (d < best) { second = best; best = d; bestI = (int)i; }
      else if (d < second) second = d;
    }
    if (bestI >= 0 && best <= shapeTol && (second == 99 || second - best >= 2)) hit = bestI;
  }
  // the short repeat frame many remotes send while a button is held, as in NEC
  bool shortRep = n >= 3 && n <= 5 && t[0] > 7500 && t[0] < 10500 && t[1] > 1800 && t[1] < 2800;
  if (hit >= 0) {
    if (!mapFiredThisPress || (rules[hit].hold && now - mapLastMs >= 150)) {
      mapFiredThisPress = true;
      // a button that was just answered is not answered again by the same long press split in two
      if (!rules[hit].hold && now - rules[hit].lastFire < 200 && rules[hit].lastFire != 0) {
        mapLastRule = hit;
      } else {
        mapPending = hit;
        mapPendingSince = now;
        mapLastRule = hit;
        mapLastMs = now;
      }
    }
  } else if (first) {
    mapLastRule = -1;
  } else if (shortRep && mapLastRule >= 0 && rules[mapLastRule].hold && now - mapLastMs >= 150) {
    mapPending = mapLastRule;
    mapPendingSince = now;
    mapLastMs = now;
  }
}

static void mapRun();

static void commitFrame(const uint16_t* t, uint16_t n) {
  if (n < 3 || (n < 9 && t[0] < 1500)) {
    noiseCount++;
    return;
  }
  uint32_t now = millis();
  if (cap.valid && (now - lastFrameMs) < 250) {
    cap.burst++;
  } else {
    cap.burst = 1;
    isNew = frameMatch(t, n, refT, refN) == 0 && frameMatch(refT, refN, t, n) == 0 && shapeDist(t, n, refT, refN) > shapeTol;
    if (isNew) newCount++;
    if (isNew || n >= refN) {  // keep the longest copy as the reference, a cut off frame must not replace a whole one
      memcpy(refT, t, n * sizeof(uint16_t));
      refN = n;
    }
  }
  lastFrameMs = now;
  mapOnFrame(t, n, cap.burst == 1);
  memcpy(cap.t, t, n * sizeof(uint16_t));
  cap.n = n;
  cap.total = 0;
  for (uint16_t i = 0; i < n; i++) cap.total += cap.t[i];
  cap.seq++;
  cap.valid = true;
  dirty = true;

  if (bleConnected || (wifiUp && millis() - httpLastPoll < 8000)) {
    String s;
    s.reserve(n * 6 + 24);
    s += "RX ";
    s += n;
    for (uint16_t i = 0; i < n; i++) {
      s += ' ';
      s += cap.t[i];
    }
    emit(s);
  }
}

// Turn one receiver's raw symbols into mark and space times, then listen again.
static void rxTake(int src) {
  rmt_symbol_word_t* buf = src ? rxBuf2 : rxBuf;
  size_t rxLen = src ? rxNum2 : rxNum;
  bool markLevel = !(src ? idleLevel2 : idleLevel);
  RxCap& c = rxCap[src];
  uint16_t n = 0;
  for (size_t i = 0; i < rxLen && n < MAX_T; i++) {
    rmt_symbol_word_t s = buf[i];
    if (s.duration0 == 0) break;
    if (!(n == 0 && (bool)s.level0 != markLevel)) c.t[n++] = s.duration0;
    if (s.duration1 == 0 || n >= MAX_T) break;
    if (!(n == 0 && (bool)s.level1 != markLevel)) c.t[n++] = s.duration1;
  }
  c.n = n;
  c.at = millis();
  c.has = n > 0;
  if (src) rx2Arm(); else rxArm();
}

static uint32_t capSpan(const RxCap& c) {
  uint32_t s = 0;
  for (uint16_t i = 0; i < c.n; i++) s += c.t[i];
  return s;
}

static void rxPoll() {
  if (rxUp && rxDone) rxTake(0);
  if (rx2Up && rxDone2) rxTake(1);
  if (!rxCap[0].has && !rxCap[1].has) return;
  // Two receivers hear the same press. Give the slower one a moment to finish, then keep the one that saw more.
  if (!(rxCap[0].has && rxCap[1].has)) {
    const RxCap& only = rxCap[0].has ? rxCap[0] : rxCap[1];
    if (rx2Up && millis() - only.at < MERGE_MS) return;
  }
  int pick = 0;
  if (!rxCap[0].has) pick = 1;
  else if (rxCap[1].has) {
    if (rxCap[1].n > rxCap[0].n) pick = 1;
    else if (rxCap[1].n == rxCap[0].n && capSpan(rxCap[1]) > capSpan(rxCap[0])) pick = 1;
  }
  uint16_t n = rxCap[pick].n;
  memcpy(tmpT, rxCap[pick].t, n * sizeof(uint16_t));
  rxCap[0].has = false;
  rxCap[1].has = false;
  if ((int32_t)(millis() - rxIgnoreUntil) < 0) return;  // our own transmission
  lastRxAnyMs = millis();
  lastSrc = pick;
  srcWins[pick]++;

  // Drop marks shorter than minMarkUs by folding them into the surrounding space.
  // A folded space as long as the frame end gap splits one capture into separate frames.
  uint32_t gapUs = GAP_US[gapIdx];
  uint32_t pending = 0;
  uint16_t fn = 0;
  for (uint16_t i = 0; i < n; i++) {
    bool mark = (i % 2 == 0);
    if (!mark || tmpT[i] < minMarkUs) {
      pending += tmpT[i];
      continue;
    }
    if (fn > 0 && pending >= gapUs) {
      commitFrame(frT, fn);
      fn = 0;
    }
    if (fn > 0) frT[fn++] = pending > 65535 ? 65535 : pending;
    if (fn < MAX_T) frT[fn++] = tmpT[i];
    pending = 0;
  }
  if (fn > 0) commitFrame(frT, fn);
}

// ---------- IR send ----------

static bool txInit() {
  rmt_tx_channel_config_t c = {};
  c.gpio_num = (gpio_num_t)IR_TX_PIN;
  c.clk_src = RMT_CLK_SRC_DEFAULT;
  c.resolution_hz = 1000000;
  c.mem_block_symbols = 128;
  c.trans_queue_depth = 1;
  if (rmt_new_tx_channel(&c, &txChan) != ESP_OK) return false;
  rmt_copy_encoder_config_t ec = {};
  if (rmt_new_copy_encoder(&ec, &copyEnc) != ESP_OK) return false;
  if (rmt_enable(txChan) != ESP_OK) return false;
  gpio_set_drive_capability((gpio_num_t)IR_TX_PIN, GPIO_DRIVE_CAP_3);  // strongest the pin offers
  // The M5 IR module's LED gets the very same signal from the same transmitter, so both fire in perfect step.
  // The pin matrix copies the output signal the driver gave the built-in LED onto the second pin.
  uint32_t sig = GPIO.func_out_sel_cfg[IR_TX_PIN].func_sel;
  esp_rom_gpio_pad_select_gpio(IR_TX2_PIN);
  gpio_set_direction((gpio_num_t)IR_TX2_PIN, GPIO_MODE_OUTPUT);
  gpio_set_drive_capability((gpio_num_t)IR_TX2_PIN, GPIO_DRIVE_CAP_3);
  esp_rom_gpio_connect_out_signal(IR_TX2_PIN, sig, false, false);
  tx2Up = true;
  return true;
}

// ---------- Broadlink output (RM4 mini over Wi-Fi) ----------
// Local protocol as python-broadlink does it: hello, auth, then AES-CBC command packets on UDP port 80.
// Checked against a real RM4 mini: hello, auth and send all worked from Python with these exact layouts.

static uint8_t blMode = 0;          // 0 this device only, 1 Broadlink only, 2 both
static String blIp;
static uint8_t blState = 0;         // 0 off, 1 ready, 2 cannot reach, 3 locked in the Broadlink app
static uint16_t blType = 0;
static uint8_t blMac[6];
static uint8_t blKey[16];
static uint32_t blId = 0;
static uint16_t blCount = 0x8000;
static uint32_t blLastAck = 0;
static uint32_t blPendingAt = 0;
static WiFiUDP blUdp;
static bool blUdpUp = false;
static uint8_t blOut[1500];
static uint8_t blPlain[1300];
static const uint8_t BL_KEY0[16] = {0x09, 0x76, 0x28, 0x34, 0x3f, 0xe9, 0x9e, 0x23, 0x76, 0x5c, 0x15, 0x13, 0xac, 0xcf, 0x8b, 0x02};
static const uint8_t BL_IV[16] = {0x56, 0x2e, 0x17, 0x99, 0x6d, 0x09, 0x3d, 0x28, 0xdd, 0xb3, 0xba, 0x69, 0x5a, 0x2e, 0x6f, 0x58};

static void blSave() {
  File f = LittleFS.open("/bl.txt", "w");
  if (!f) return;
  f.printf("%d %s\n", blMode, blIp.c_str());
  f.close();
}

static void blLoad() {
  File f = LittleFS.open("/bl.txt", "r");
  if (!f) return;
  String l = f.readStringUntil('\n');
  f.close();
  l.trim();
  int sp = l.indexOf(' ');
  if (sp < 1) return;
  int m = l.substring(0, sp).toInt();
  if (m >= 0 && m <= 2) blMode = m;
  blIp = l.substring(sp + 1);
  blIp.trim();
}

// A fresh socket each time we log in, so a Wi-Fi drop and rejoin can never leave a dead one behind.
static void blSock() {
  if (blUdpUp) blUdp.stop();
  blUdp.begin(49321);
  blUdpUp = true;
}

static void blDrain() {
  uint8_t junk[64];
  while (blUdp.parsePacket() > 0) blUdp.read(junk, sizeof junk);
}

static size_t blHello(uint8_t* p) {
  memset(p, 0, 0x30);
  p[0x0C] = 0xEA; p[0x0D] = 0x07;  // 2026, the device does not check the clock
  p[0x0F] = 12; p[0x10] = 26; p[0x11] = 7; p[0x12] = 8; p[0x13] = 11;
  p[0x26] = 6;
  uint16_t c = 0xBEAF;
  for (int i = 0; i < 0x30; i++) c += p[i];
  p[0x20] = c & 0xFF; p[0x21] = c >> 8;
  return 0x30;
}

static size_t blBuild(uint16_t cmd, const uint8_t* pl, size_t plen, const uint8_t* key, uint8_t* out) {
  size_t pad = (plen + 15) & ~(size_t)15;
  if (pad > sizeof blPlain || 0x38 + pad > sizeof blOut) return 0;
  memset(out, 0, 0x38);
  const uint8_t magic[8] = {0x5a, 0xa5, 0xaa, 0x55, 0x5a, 0xa5, 0xaa, 0x55};
  memcpy(out, magic, 8);
  out[0x24] = blType & 0xFF; out[0x25] = blType >> 8;
  out[0x26] = cmd & 0xFF; out[0x27] = cmd >> 8;
  blCount = ((blCount + 1) | 0x8000) & 0xFFFF;
  out[0x28] = blCount & 0xFF; out[0x29] = blCount >> 8;
  memcpy(out + 0x2A, blMac, 6);
  for (int i = 0; i < 4; i++) out[0x30 + i] = (blId >> (8 * i)) & 0xFF;
  uint16_t pc = 0xBEAF;
  for (size_t i = 0; i < plen; i++) pc += pl[i];
  out[0x34] = pc & 0xFF; out[0x35] = pc >> 8;
  memcpy(blPlain, pl, plen);
  memset(blPlain + plen, 0, pad - plen);
  uint8_t iv[16];
  memcpy(iv, BL_IV, 16);
  mbedtls_aes_context ctx;
  mbedtls_aes_init(&ctx);
  mbedtls_aes_setkey_enc(&ctx, key, 128);
  mbedtls_aes_crypt_cbc(&ctx, MBEDTLS_AES_ENCRYPT, pad, iv, blPlain, out + 0x38);
  mbedtls_aes_free(&ctx);
  uint16_t ck = 0xBEAF;
  for (size_t i = 0; i < 0x38 + pad; i++) ck += out[i];
  out[0x20] = ck & 0xFF; out[0x21] = ck >> 8;
  return 0x38 + pad;
}

static int blExchange(const uint8_t* pkt, size_t n, const IPAddress& to, uint8_t* resp, size_t cap, uint32_t waitMs) {
  blDrain();
  blUdp.beginPacket(to, 80);
  blUdp.write(pkt, n);
  blUdp.endPacket();
  uint32_t t0 = millis();
  while (millis() - t0 < waitMs) {
    int k = blUdp.parsePacket();
    if (k > 0) return blUdp.read(resp, k < (int)cap ? k : (int)cap);
    delay(2);
  }
  return 0;
}

static bool blConnect() {
  blState = 2;
  if (!wifiUp || !blIp.length()) { blState = 0; return false; }
  IPAddress ip;
  if (!ip.fromString(blIp)) return false;
  blSock();
  uint8_t pkt[0x60], resp[512];
  int r = 0;
  for (int t = 0; t < 3 && r < 0x80; t++) {
    size_t n = blHello(pkt);
    r = blExchange(pkt, n, ip, resp, sizeof resp, 700);
  }
  if (r < 0x80) return false;
  blType = resp[0x34] | (resp[0x35] << 8);
  memcpy(blMac, resp + 0x3A, 6);
  if (resp[0x7F]) { blState = 3; return false; }
  uint8_t ap[0x50];
  memset(ap, 0, sizeof ap);
  memset(ap + 0x04, 0x31, 16);
  ap[0x1E] = 1; ap[0x2D] = 1;
  memcpy(ap + 0x30, "Test 1", 6);
  blId = 0;
  size_t n = blBuild(0x65, ap, sizeof ap, BL_KEY0, blOut);
  r = 0;
  for (int t = 0; t < 3 && r < 0x48; t++) r = blExchange(blOut, n, ip, resp, sizeof resp, 700);
  if (r < 0x48 || resp[0x22] || resp[0x23]) return false;
  size_t len = ((size_t)(r - 0x38) / 16) * 16;
  uint8_t dec[512];
  uint8_t iv[16];
  memcpy(iv, BL_IV, 16);
  mbedtls_aes_context ctx;
  mbedtls_aes_init(&ctx);
  mbedtls_aes_setkey_dec(&ctx, BL_KEY0, 128);
  mbedtls_aes_crypt_cbc(&ctx, MBEDTLS_AES_DECRYPT, len, iv, resp + 0x38, dec);
  mbedtls_aes_free(&ctx);
  blId = dec[0] | (dec[1] << 8) | (dec[2] << 16) | ((uint32_t)dec[3] << 24);
  memcpy(blKey, dec + 4, 16);
  blState = 1;
  blLastAck = millis();
  return true;
}

// Our mark and space times become the Broadlink pulse list: 32.84 us ticks, one byte, or 0 then two bytes.
static size_t blEncode(const uint16_t* t, uint16_t n, uint8_t rep, uint16_t gapMs, uint8_t* out, size_t cap) {
  size_t o = 4;
  auto put = [&](uint32_t us) -> bool {
    uint32_t u = (uint32_t)((uint64_t)us * 100 / 3284);
    if (u < 1) u = 1;
    if (u > 65535) u = 65535;
    if (u < 256) { if (o + 1 > cap) return false; out[o++] = (uint8_t)u; }
    else { if (o + 3 > cap) return false; out[o++] = 0; out[o++] = u >> 8; out[o++] = u & 0xFF; }
    return true;
  };
  for (uint8_t r = 0; r < rep; r++) {
    for (uint16_t i = 0; i < n; i++) {
      uint32_t d = t[i];
      bool last = (i == n - 1);
      if (last && (i % 2 == 1)) {  // the closing space doubles as the pause before the next frame
        uint32_t want = (r + 1 < rep) ? (uint32_t)gapMs * 1000 : 0;
        if (want > d) d = want;
      }
      if (!put(d)) return 0;
      if (last && (i % 2 == 0)) {  // a frame that ends on a pulse gets a closing space
        if (!put((r + 1 < rep) ? (gapMs ? (uint32_t)gapMs * 1000 : 5000) : 2000)) return 0;
      }
    }
  }
  size_t body = o - 4;
  out[0] = 0x26; out[1] = 0; out[2] = body & 0xFF; out[3] = body >> 8;
  return o;
}

// The Broadlink may sit right next to this device, so its own output is not mistaken for a remote being pressed.
static void blQuiet(const uint16_t* t, uint16_t n, uint8_t rep, uint16_t gapMs) {
  uint32_t us = 0;
  for (uint16_t i = 0; i < n; i++) us += t[i];
  uint32_t ms = (us / 1000 + gapMs) * rep + 250;
  rxIgnoreUntil = millis() + ms;
}

static bool blSendIr(const uint16_t* t, uint16_t n, uint8_t rep, uint16_t gapMs) {
  if (!blIp.length() || !wifiUp || n < 2) return false;
  IPAddress ip;
  if (!ip.fromString(blIp)) return false;
  bool fresh = blState != 1 || millis() - blLastAck > 30000;  // after a quiet spell, check the Broadlink really took it
  if (blState != 1 && !blConnect()) { emit("BLINK fail"); return false; }
  static uint8_t ir[1100];
  size_t irLen = blEncode(t, n, rep, gapMs, ir, sizeof ir);
  if (!irLen) { emit("BLINK toolong"); return false; }
  uint8_t pl[1110];
  size_t plen = irLen + 6;
  pl[0] = (irLen + 4) & 0xFF; pl[1] = (irLen + 4) >> 8;
  pl[2] = 2; pl[3] = 0; pl[4] = 0; pl[5] = 0;
  memcpy(pl + 6, ir, irLen);
  size_t n2 = blBuild(0x6A, pl, plen, blKey, blOut);
  if (!n2) return false;
  uint8_t resp[512];
  if (fresh) {
    int r = blExchange(blOut, n2, ip, resp, sizeof resp, 350);
    if (r < 0x30 || resp[0x22] || resp[0x23]) {  // stale login, log in again once
      if (!blConnect()) { emit("BLINK fail"); return false; }
      n2 = blBuild(0x6A, pl, plen, blKey, blOut);
      r = blExchange(blOut, n2, ip, resp, sizeof resp, 350);
      if (r < 0x30 || resp[0x22] || resp[0x23]) { blState = 2; emit("BLINK fail"); return false; }
    }
    blLastAck = millis();
    blQuiet(t, n, rep, gapMs);
    return true;
  }
  blDrain();
  blUdp.beginPacket(ip, 80);
  blUdp.write(blOut, n2);
  blUdp.endPacket();
  blPendingAt = millis();
  blQuiet(t, n, rep, gapMs);
  return true;
}

// Quick sends do not wait for the answer. If it never comes the next send logs in again.
static uint32_t blRetryAt = 0;
static void blPoll() {
  // Log in by ourselves once Wi-Fi is up, and again every minute while the Broadlink is not answering.
  if (blMode > 0 && blIp.length() && wifiUp && !blPendingAt && (blState == 0 || (blState == 2 && millis() - blRetryAt > 60000))) {
    blRetryAt = millis();
    blConnect();
    emit(String("BL ") + blMode + " " + blIp + " " + blState);
    return;
  }
  if (!blPendingAt) return;
  int k = blUdp.parsePacket();
  if (k > 0) {
    uint8_t resp[128];
    int r = blUdp.read(resp, k < (int)sizeof resp ? k : (int)sizeof resp);
    blPendingAt = 0;
    if (r >= 0x30 && !resp[0x22] && !resp[0x23]) blLastAck = millis();
    else { blState = 2; emit("BLINK lost"); }
  } else if (millis() - blPendingAt > 900) {
    blPendingAt = 0;
    blState = 2;
    emit("BLINK lost");
  }
}

static void blFind() {
  if (!wifiUp) { bleSend("BLFOUND none"); return; }
  blSock();
  uint8_t pkt[0x30];
  size_t n = blHello(pkt);
  blDrain();
  IPAddress bc = WiFi.broadcastIP();
  for (int t = 0; t < 2; t++) {
    blUdp.beginPacket(bc, 80);
    blUdp.write(pkt, n);
    blUdp.endPacket();
    delay(30);
  }
  uint32_t t0 = millis();
  String seen;
  int found = 0;
  while (millis() - t0 < 2500) {
    int k = blUdp.parsePacket();
    if (k >= 0x80) {
      uint8_t resp[256];
      int r = blUdp.read(resp, k < (int)sizeof resp ? k : (int)sizeof resp);
      String ip = blUdp.remoteIP().toString();
      if (r >= 0x80 && seen.indexOf(ip) < 0) {
        seen += ip + " ";
        char name[33];
        int ni = 0;
        for (int i = 0x40; i < r && ni < 32 && resp[i]; i++) name[ni++] = (resp[i] >= 32 && resp[i] < 127) ? resp[i] : '_';
        name[ni] = 0;
        bleSend(String("BLFOUND ") + ip + " " + name);
        found++;
      }
    } else if (k > 0) {
      uint8_t junk[64];
      blUdp.read(junk, sizeof junk);
    }
    delay(5);
  }
  if (!found) bleSend("BLFOUND none");
}

static void blSet(const String& l) {  // BL <mode> <ip>
  int m = 0;
  char ipb[40] = "";
  int got = sscanf(l.c_str() + 2, "%d %39s", &m, ipb);
  if (got < 1 || m < 0 || m > 2) { bleSend("ERR bl"); return; }
  String ip = got >= 2 ? String(ipb) : blIp;
  IPAddress test;
  if (m > 0 && !test.fromString(ip)) { bleSend("ERR bl address"); return; }
  blMode = m;
  blIp = ip;
  blSave();
  blState = 0;
  if (blMode > 0) blConnect();
  bleSend(String("BL ") + blMode + " " + blIp + " " + blState);
}

static bool irSend(uint32_t hz, const uint16_t* t, uint16_t n, uint8_t rep, uint16_t gapMs) {
  if (blMode > 0) {
    bool ok = blSendIr(t, n, rep, gapMs);
    if (blMode == 1) {
      if (ok) { txCount++; dirty = true; }
      return ok;
    }
  }
  if (!txUp || n < 2) return false;
  static uint8_t dutyApplied = 0;
  if (hz != txCarrier || txDuty != dutyApplied) {
    rmt_carrier_config_t cc = {};
    cc.frequency_hz = hz;
    cc.duty_cycle = txDuty / 100.0f;
    if (rmt_apply_carrier(txChan, &cc) != ESP_OK) return false;
    txCarrier = hz;
    dutyApplied = txDuty;
  }
  size_t m = 0;
  for (uint16_t i = 0; i < n && m < MAX_T; i += 2) {
    uint32_t mark = t[i] > 32767 ? 32767 : t[i];
    uint32_t space = (i + 1 < n) ? t[i + 1] : 200;
    if (space > 32767) space = 32767;
    if (space == 0) space = 1;
    txSyms[m].level0 = 1;
    txSyms[m].duration0 = mark;
    txSyms[m].level1 = 0;
    txSyms[m].duration1 = space;
    m++;
  }
  rmt_transmit_config_t tc = {};
  rxIgnoreUntil = millis() + 4000;
  bool ok = true;
  for (uint8_t r = 0; r < rep; r++) {
    if (rmt_transmit(txChan, copyEnc, txSyms, m * sizeof(rmt_symbol_word_t), &tc) != ESP_OK) {
      ok = false;
      break;
    }
    rmt_tx_wait_all_done(txChan, 2000);
    if (r + 1 < rep && gapMs) delay(gapMs);
  }
  rxIgnoreUntil = millis() + 150;
  if (ok) {
    txCount++;
    dirty = true;
  }
  return ok;
}

static void mapRun() {
  int i = mapPending;
  if (i < 0 || i >= (int)rules.size()) { mapPending = -1; return; }
  // wait until the remote has gone quiet, sending while it still blasts garbles both
  uint32_t need = rules[i].hold ? 30 : sendWaitMs;
  if (need && millis() - lastRxAnyMs < need && millis() - mapPendingSince < 2500) return;
  mapPending = -1;
  Rule& r = rules[i];
  r.lastFire = millis();
  bool allOk = true;
  snprintf(lastTxName, sizeof lastTxName, "%s", r.name);
  for (size_t k = 0; k < r.acts.size(); k++) {
    Act& a = r.acts[k];
    if (k) delay(40);
    if (a.type == 'I') {
      uint8_t rep = a.rep < sendRepeats ? sendRepeats : a.rep;
      uint16_t gap = (rep > 1 && a.gap < 40) ? 40 : a.gap;  // frames need a pause between them
      if (!irSend(a.hz, a.t, a.n, rep, gap)) allOk = false;
    }
    else if (a.type == 'K') { if (kbTap((uint8_t)a.a, (uint8_t)a.b)) txCount++; else allOk = false; }
    else if (a.type == 'C') { if (consTap((uint16_t)a.a)) txCount++; else allOk = false; }
    else if (a.type == 'M') { if (mouseSend((uint8_t)a.a, (int8_t)a.b, (int8_t)a.c, (int8_t)a.d)) txCount++; else allOk = false; }
  }
  mapLastMs = millis();
  emit(String("FIRED ") + r.name + (allOk ? "" : " FAILED"));
  dirty = true;
}

// ---------- commands from the web tool ----------

static File mapFile;
static int mapCount = 0;

static void cmdMap(const String& l) {
  if (l == "MAPBEGIN") {
    if (mapFile) mapFile.close();
    mapFile = LittleFS.open("/map.tmp", "w");
    mapCount = 0;
    if (!mapFile) bleSend("ERR map");
  } else if (l.startsWith("MAPR ")) {
    if (mapFile) {
      mapFile.print(l.c_str() + 5);
      mapFile.print('\n');
      mapCount++;
    }
  } else if (l == "MAPEND") {
    if (!mapFile) { bleSend("ERR map"); return; }
    mapFile.close();
    LittleFS.remove("/map.txt");
    LittleFS.rename("/map.tmp", "/map.txt");
    rulesLoad();
    bleSend(String("MAPSAVED ") + rules.size());
    dirty = true;
  }
}

static void cmdSend(const String& l) {
  const char* p = l.c_str() + 5;
  int k = 0;
  while (*p == ' ') p++;
  while (*p && *p != ' ' && k < 23) lastTxName[k++] = *p++;
  lastTxName[k] = 0;
  char* e;
  uint32_t hz = strtoul(p, &e, 10);
  p = e;
  uint32_t rep = strtoul(p, &e, 10);
  p = e;
  uint32_t gap = strtoul(p, &e, 10);
  p = e;
  uint16_t n = 0;
  while (n < MAX_T) {
    while (*p == ' ') p++;
    if (!*p) break;
    sendT[n++] = (uint16_t)strtoul(p, &e, 10);
    if (e == p) break;
    p = e;
  }
  if (hz < 20000 || hz > 60000 || rep < 1 || rep > 20 || gap > 2000 || n < 2) {
    bleSend("ERR send");
    return;
  }
  bleSend(irSend(hz, sendT, n, rep, gap) ? String("SENT ") + lastTxName : String("ERR send"));
}

static void cmdKeyboard(const String& l) {
  int a = 0, b = 0, c = 0, d = 0;
  bool ok = false;
  const char* p = l.c_str();
  if (l.startsWith("KEY ") && sscanf(p + 4, "%d %d", &a, &b) == 2) ok = kbTap((uint8_t)a, (uint8_t)b);
  else if (l.startsWith("CONS ") && sscanf(p + 5, "%d", &a) == 1) ok = consTap((uint16_t)a);
  else if (l.startsWith("MOUSE ") && sscanf(p + 6, "%d %d %d %d", &a, &b, &c, &d) == 4) ok = mouseSend((uint8_t)a, (int8_t)b, (int8_t)c, (int8_t)d);
  if (ok) {
    snprintf(lastTxName, sizeof lastTxName, "Keyboard");
    txCount++;
    dirty = true;
  }
  bleSend(ok ? String("SENT key") : String("ERR keyboard not paired"));
}

static void otaFail(const char* why) {
  Update.abort();
  otaActive = false;
  otaAbort = false;
  otaOverflow = false;
  if (otaBuf) { free(otaBuf); otaBuf = nullptr; }
  bleSend(String("ERR ota ") + why);
  dirty = true;
}

static void cmdOta(const String& l) {
  uint32_t size = strtoul(l.c_str() + 4, nullptr, 10);
  if (otaActive) otaFail("restarted");
  if (size < 100000 || size > 0x330000) { bleSend("ERR ota size"); return; }
  otaBuf = (uint8_t*)malloc(OTA_RING);
  if (!otaBuf) { bleSend("ERR ota memory"); return; }
  if (!Update.begin(size)) {
    free(otaBuf);
    otaBuf = nullptr;
    bleSend(String("ERR ota begin ") + Update.errorString());
    return;
  }
  otaSize = size;
  otaHead = otaTail = otaAckAt = 0;
  otaAbort = otaOverflow = false;
  otaLastMs = millis();
  otaActive = true;
  wake();
  dirty = true;
  bleSend("OTAGO");
}

static void otaPump() {
  if (!otaActive) return;
  if (otaAbort) { otaFail("link lost"); return; }
  if (otaOverflow) { otaFail("overflow"); return; }
  if (millis() - otaLastMs > 30000) { otaFail("timeout"); return; }  // never stay stuck on the update screen
  while (otaHead != otaTail) {
    uint32_t avail = otaHead - otaTail;
    uint32_t pos = otaTail % OTA_RING;
    uint32_t n = avail;
    if (n > OTA_RING - pos) n = OTA_RING - pos;
    if (n > 4096) n = 4096;
    if (Update.write(otaBuf + pos, n) != n) { otaFail("flash write"); return; }
    otaTail = otaTail + n;
    if (otaTail - otaAckAt >= 4096 || otaTail >= otaSize) {
      otaAckAt = otaTail;
      bleSend(String("OTA+ ") + otaTail);
      dirty = true;
    }
    if (otaTail >= otaSize) {
      if (Update.end(true)) {
        otaActive = false;
        free(otaBuf);
        otaBuf = nullptr;
        otaDoneMs = millis();
        bleSend("OTADONE");
        dirty = true;
      } else {
        otaFail("bad image");
      }
      return;
    }
  }
}

static void wifiSet(const String& ssid, const String& pass);
static void wifiForget();
static void wifiScan();

static void cmdSave(const String& l) {
  File f = LittleFS.open("/settings.json", "w");
  if (!f) {
    bleSend("ERR save");
    return;
  }
  size_t len = l.length() - 5;
  f.write((const uint8_t*)l.c_str() + 5, len);
  f.close();
  bleSend(String("SAVED ") + len);
}

static void cmdLoad() {
  File f = LittleFS.open("/settings.json", "r");
  if (!f) {
    bleSend("FILE");
    return;
  }
  String s = f.readString();
  f.close();
  bleSend("FILE " + s);
}

static void handleLine(String& l) {
  l.trim();
  bool viaBle = replySink == nullptr;
  if (viaBle && l.length() && !l.startsWith("HELLO") && l != "PING") {
    if (!windowOpen()) { bleSend("ERR locked"); return; }
    if ((int32_t)(openUntil - (millis() + 120000)) < 0) openUntil = millis() + 120000;
  }
  if (l.startsWith("HELLO")) {
    bleSend(String("HELLO super-remote ") + FW_VERSION + " p" + PROTO_VERSION + " bat=" + batt + " in=" + (rxUp ? 1 : 0) + " out=" + (txUp ? 1 : 0) + " in2=" + (rx2Up ? 1 : 0) + " out2=" + (tx2Up ? 1 : 0) + " src=" + srcWins[0] + "," + srcWins[1] + " kb=" + (kbConnected ? 1 : 0) + " ota=1 map=1 ble=" + (bleKbMode ? "kb" : "setup") + " tune=" + tolAbs + "," + tolPct + "," + (prefixOn ? 1 : 0) + "," + minMarkUs + "," + gapIdx + "," + shapeTol + "," + sendWaitMs + "," + sendRepeats + "," + txDuty + " bl=" + blMode + "," + (blIp.length() ? blIp : String("-")) + "," + blState + " wifion=" + (wifiOn ? 1 : 0) + " lock=1 open=" + (windowOpen() ? 1 : 0) + " wifi=" + (wifiUp ? wifiIp : String("0")) + " rules=" + rules.size());
  } else if (l.startsWith("SEND ")) {
    cmdSend(l);
  } else if (l.startsWith("KEY ") || l.startsWith("CONS ") || l.startsWith("MOUSE ")) {
    cmdKeyboard(l);
  } else if (l.startsWith("MODE ")) {
    bool kb = l.endsWith("kb");
    if (l.endsWith("kb") || l.endsWith("setup")) {
      bleKbMode = kb;
      bleModeSaved = true;
      prefsSave();
      bleSend(String("MODE ") + (kb ? "kb" : "setup"));
      restartAt = millis() + 700;  // Bluetooth changes shape, so the stick restarts
    } else {
      bleSend("ERR mode");
    }
  } else if (l.startsWith("TUNE ")) {
    int ta, tp, px, mm, g, sh = shapeTol, sw = sendWaitMs, sr = sendRepeats, td = txDuty;
    int got = sscanf(l.c_str() + 5, "%d %d %d %d %d %d %d %d %d", &ta, &tp, &px, &mm, &g, &sh, &sw, &sr, &td);
    if (got >= 5 && ta >= 50 && ta <= 1000 && tp >= 0 && tp <= 50 && mm >= 100 && mm <= 1000 && g >= 0 && g < 3 && sh >= 0 && sh <= 8 && sw >= 0 && sw <= 1000 && sr >= 1 && sr <= 6 && td >= 20 && td <= 66) {
      tolAbs = ta; tolPct = tp; prefixOn = px != 0; minMarkUs = mm; gapIdx = g; shapeTol = sh; sendWaitMs = sw; sendRepeats = sr; txDuty = td;
      prefsSave();
      bleSend("TUNED");
    } else {
      bleSend("ERR tune");
    }
  } else if (l == "TOKEN") {
    bleSend(viaBle ? String("TOKEN ") + apiKey : String("ERR no"));
  } else if (l == "BLFIND") {
    blFind();
  } else if (l.startsWith("BL ")) {
    blSet(l);
  } else if (l == "WIFISCAN") {
    wifiScan();
  } else if (l == "WIFIOFF") {
    wifiForget();
  } else if (l.startsWith("WIFI")) {
    if (l.length() > 5) {
      int k = l.indexOf('|', 5);
      wifiSet(k < 0 ? l.substring(5) : l.substring(5, k), k < 0 ? String("") : l.substring(k + 1));
    } else {
      bleSend(wifiUp ? String("WIFI up ") + wifiIp : String("WIFI ") + wifiMsg);
    }
  } else if (l.startsWith("MAP")) {
    cmdMap(l);
  } else if (l.startsWith("OTA ")) {
    cmdOta(l);
  } else if (l.startsWith("SAVE ")) {
    cmdSave(l);
  } else if (l == "LOAD") {
    cmdLoad();
  } else if (l == "PING") {
    bleSend("PONG");
  } else if (l.length()) {
    bleSend("ERR unknown");
  }
}

static void pollCommands() {
  for (int i = 0; i < 4; i++) {
    String line;
    xSemaphoreTake(inMux, portMAX_DELAY);
    bool have = !inLines.empty();
    if (have) {
      line = inLines.front();
      inLines.pop_front();
    }
    xSemaphoreGive(inMux);
    if (!have) return;
    handleLine(line);
  }
}

// ---------- drawing ----------

static void txt(int x, int y, uint8_t sz, uint16_t col, const char* s) {
  cv.setTextSize(sz);
  cv.setTextColor(col);
  cv.setCursor(x, y);
  cv.print(s);
}

static void drawHeader(const char* title) {
  txt(2, 2, 1, C_DIM, title);
  if (batt >= 0) {
    char b[8];
    snprintf(b, sizeof b, "%d%%", batt);
    txt(W - 2 - (int)strlen(b) * 6, 2, 1, C_DIM, b);
  }
  cv.drawFastHLine(0, 12, W, C_LINE);
}

static void drawRowC(int y, const char* label, const char* val, uint16_t col) {
  txt(2, y, 1, C_DIM, label);
  txt(60, y, 1, col, val);
}

static void drawMain() {
  drawHeader("REVENANT v" FW_VERSION);
  if (cap.valid) cv.fillRect(0, 13, W, 4, isNew ? C_NEW : C_SAME);
  char b[40];
  drawRowC(24, "Bluetooth", bleKbMode ? "Keyboard" : "Setup", C_BT);
  drawRowC(36, "IR In", !rxUp ? "FAILED" : rx2Up ? "Both Ready" : "Ready", rxUp ? C_TEXT : C_BAD);
  drawRowC(48, "IR Out", !txUp ? "FAILED" : tx2Up ? "Both Ready" : "Ready", txUp ? C_TEXT : C_BAD);
  if (bleKbMode) drawRowC(60, "BT Link", kbConnected ? "Paired" : "Waiting", kbConnected ? C_NEW : C_SAME);
  else drawRowC(60, "BT Link", !bleConnected ? "Waiting" : windowOpen() ? "Open" : "Press Front", !bleConnected ? C_SAME : windowOpen() ? C_NEW : C_HI);
  snprintf(b, sizeof b, "%u mapped", (unsigned)rules.size());
  drawRowC(72, "Rules", b, rules.empty() ? C_DIM : C_NEW);
  drawRowC(84, "Wi-Fi", wifiUp ? wifiIp.c_str() : wifiMsg, wifiUp ? C_NEW : C_DIM);

  txt(2, 104, 1, C_DIM, "Last Heard");
  if (!cap.valid) {
    txt(2, 116, 2, C_DIM, "Nothing");
  } else {
    Guess g = guessProtocol(cap);
    uint16_t nc = isNew ? C_NEW : C_SAME;
    txt(2, 116, 2, nc, g.name);
    snprintf(b, sizeof b, "#%lu %s %lu", (unsigned long)cap.seq, isNew ? "NEW" : "SAME", (unsigned long)newCount);
    drawRowC(138, "Frame", b, nc);
    snprintf(b, sizeof b, "%u  %d bits", cap.n, g.bits);
    if (g.bits < 0) snprintf(b, sizeof b, "%u", cap.n);
    drawRowC(150, "Timings", b, C_TEXT);
  }

  txt(2, 172, 1, C_DIM, "Last Sent");
  if (txCount == 0) {
    txt(2, 184, 1, C_DIM, "Nothing");
  } else {
    txt(2, 184, 1, C_TEXT, lastTxName);
    snprintf(b, sizeof b, "Total %lu", (unsigned long)txCount);
    txt(2, 196, 1, C_DIM, b);
  }
  txt(2, H - 21, 1, C_DIM, "Front Echo Right Menu");
}

static void drawMenu() {
  drawHeader("MENU");
  const char* labels[MENU_ITEMS] = {"Brightness", "Screen sleep", "Wi-Fi", "Bluetooth", "Frame end", "Pairings", ""};
  const char* values[MENU_ITEMS] = {BRIGHT_NAME[brIdx], SLEEP_NAME[sleepIdx], wifiOn ? "Wi-Fi On" : "Wi-Fi Off", restartAt ? "Restarting" : bleKbMode ? "Keyboard" : "Setup", GAP_NAME[gapIdx], (forgetShownMs && millis() - forgetShownMs < 2500) ? "Forgotten" : "Forget All", "Close"};
  for (int i = 0; i < MENU_ITEMS; i++) {
    int y = 20 + i * 29;
    uint16_t col = (i == menuSel) ? C_HI : C_TEXT;
    if (i == menuSel) cv.fillRect(0, y, 2, 26, C_HI);
    txt(6, y, 1, C_DIM, labels[i]);
    txt(6, y + 9, 2, col, values[i]);
  }
  txt(2, H - 21, 1, C_DIM, "Right Next Front Pick");
}

static void drawOta() {
  drawHeader("UPDATING");
  int pct = otaDoneMs ? 100 : otaShow ? otaPct : (otaSize ? (int)((uint64_t)otaTail * 100 / otaSize) : 0);
  // one fixed cyan to magenta ramp along the whole bar, the fill reveals it as it grows, so the head is always the newest colour
  const int bw = W - 12;
  const int fw = bw * pct / 100;
  auto ramp = [&](int x) {
    int r = 25 + (255 - 25) * x / bw;
    int g = 242 + (43 - 242) * x / bw;
    int bl = 255 + (214 - 255) * x / bw;
    return C565(r, g, bl);
  };
  char b[24];
  snprintf(b, sizeof b, "%d%%", pct);
  txt(2, 80, 3, ramp(fw > 0 ? fw - 1 : 0), b);
  cv.drawRect(4, 120, W - 8, 12, C_LINE);
  for (int x = 0; x < fw; x++) cv.drawFastVLine(6 + x, 122, 8, ramp(x));
  txt(2, 146, 1, C_DIM, otaDoneMs ? "Restarting" : "Do not unplug");
}

// Bottom band: the loading bar colours run backwards, magenta to cyan, with black text on top.
static void drawBand() {
  const int by = H - 11;
  for (int x = 0; x < W; x++) {
    int r = 255 + (25 - 255) * x / (W - 1);
    int g = 43 + (242 - 43) * x / (W - 1);
    int bl = 214 + (255 - 214) * x / (W - 1);
    cv.drawFastVLine(x, by, 11, C565(r, g, bl));
  }
  txt((W - 7 * 6) / 2, by + 2, 1, TFT_BLACK, "woz.lol");
}

static void render() {
  cv.fillSprite(TFT_BLACK);
  cv.setFont(&fonts::Font0);
  if (otaActive || otaShow || otaDoneMs) drawOta();
  else if (menuOpen) { drawMenu(); drawBand(); }
  else { drawMain(); drawBand(); }
  cv.pushSprite(0, 0);
}

// ---------- Wi-Fi: join the home network, answer the web page, take firmware updates ----------

static WebServer server(80);
static String wifiSsid, wifiPass;
static uint32_t wifiStartMs = 0;
static uint32_t wifiRetryMs = 0;
static bool wifiTrying = false;
static bool serverStarted = false;
static bool httpOtaOk = false;
static uint32_t httpOtaTotal = 0;
static uint32_t httpOtaDone = 0;

static void keyNew() {
  char b[33];
  for (int i = 0; i < 4; i++) snprintf(b + i * 8, 9, "%08x", (unsigned)esp_random());
  apiKey = b;
  File f = LittleFS.open("/key.txt", "w");
  if (f) { f.print(apiKey); f.close(); }
}

static void keyLoad() {
  File f = LittleFS.open("/key.txt", "r");
  if (f) { apiKey = f.readString(); f.close(); apiKey.trim(); }
  if (apiKey.length() != 32) keyNew();
}

static void wifiBegin() {
  WiFi.mode(WIFI_STA);
  WiFi.setHostname("revenant");
  WiFi.begin(wifiSsid.c_str(), wifiPass.c_str());
  wifiTrying = true;
  wifiStartMs = millis();
  wifiMsg = "Joining";
  dirty = true;
}

static void wifiSet(const String& ssid, const String& pass) {
  wifiSsid = ssid;
  wifiPass = pass;
  wifiOn = true;
  File f = LittleFS.open("/wifi.txt", "w");
  if (f) {
    f.print(ssid);
    f.print('\n');
    f.print(pass);
    f.print('\n');
    f.close();
  }
  WiFi.disconnect();
  wifiUp = false;
  wifiBegin();
  bleSend("WIFI joining");
}

static void wifiPower(bool on) {
  wifiOn = on;
  if (on) {
    if (wifiSsid.length()) wifiBegin();
    else wifiMsg = "No Network";
  } else {
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
    if (serverStarted) { server.stop(); serverStarted = false; }
    wifiUp = false;
    wifiTrying = false;
    wifiMsg = "Off";
  }
  dirty = true;
}

static void wifiForget() {
  LittleFS.remove("/wifi.txt");
  wifiSsid = "";
  wifiPass = "";
  WiFi.disconnect(true);
  keyNew();  // forgetting the network also retires the old secret
  wifiUp = false;
  wifiTrying = false;
  wifiMsg = "Off";
  dirty = true;
  bleSend("WIFI off");
}

// list the networks the stick can hear, strongest first, one line each: WIFINET rssi|secured|name
static void wifiScan() {
  bool wasOff = WiFi.getMode() == WIFI_OFF;
  if (wasOff) WiFi.mode(WIFI_STA);
  int n = WiFi.scanNetworks(false, false, false, 300);
  int sent = 0;
  String seen = "|";
  for (int i = 0; i < n && sent < 24; i++) {
    String name = WiFi.SSID(i);
    if (!name.length() || seen.indexOf("|" + name + "|") >= 0) continue;
    seen += name + "|";
    bleSend(String("WIFINET ") + WiFi.RSSI(i) + "|" + (WiFi.encryptionType(i) == WIFI_AUTH_OPEN ? 0 : 1) + "|" + name);
    sent++;
  }
  WiFi.scanDelete();
  if (wasOff && !wifiTrying && !wifiUp) WiFi.mode(WIFI_OFF);
  bleSend(String("WIFIEND ") + sent);
}

static void wifiLoad() {
  File f = LittleFS.open("/wifi.txt", "r");
  if (!f) return;
  wifiSsid = f.readStringUntil('\n');
  wifiPass = f.readStringUntil('\n');
  f.close();
  wifiSsid.replace("\r", "");
  wifiPass.replace("\r", "");
}

// only the local page may read answers, and every request needs the secret
static bool originOk() {
  String o = server.header("Origin");
  return o == "http://localhost:8090" || o == "http://127.0.0.1:8090";
}

static void cors() {
  if (originOk()) {
    server.sendHeader("Access-Control-Allow-Origin", server.header("Origin"));
    server.sendHeader("Vary", "Origin");
  }
  server.sendHeader("Access-Control-Allow-Private-Network", "true");
  server.sendHeader("Access-Control-Allow-Methods", "GET, POST, OPTIONS");
  server.sendHeader("Access-Control-Allow-Headers", "X-Revenant-Key, Content-Type");
}

static void sendText(int code, const String& body) {
  cors();
  server.send(code, "text/plain", body);
}

static bool authed() {
  return apiKey.length() == 32 && server.header("X-Revenant-Key") == apiKey;
}

static bool guard() {
  if (authed()) return true;
  sendText(401, "ERR auth");
  return false;
}

static void wifiRoutes() {
  static const char* keep[] = {"X-Revenant-Key", "Origin"};
  server.collectHeaders(keep, 2);
  server.onNotFound([]() {
    if (server.method() == HTTP_OPTIONS) {
      cors();
      server.send(204);
    } else {
      sendText(404, "not found");
    }
  });
  server.on("/hello", HTTP_GET, []() {
    if (!guard()) return;
    httpLastPoll = millis();
    String out;
    replySink = &out;
    String l = "HELLO";
    handleLine(l);
    replySink = nullptr;
    sendText(200, out);
  });
  server.on("/cmd", HTTP_POST, []() {
    if (!guard()) return;
    httpLastPoll = millis();
    String body = server.arg("plain");
    String out;
    replySink = &out;
    int from = 0;
    while (from <= (int)body.length()) {
      int k = body.indexOf('\n', from);
      String line = k < 0 ? body.substring(from) : body.substring(from, k);
      from = k < 0 ? (int)body.length() + 1 : k + 1;
      if (line.length() && !line.startsWith("OTA")) handleLine(line);
    }
    replySink = nullptr;
    sendText(200, out);
  });
  server.on("/poll", HTTP_GET, []() {
    if (!guard()) return;
    httpLastPoll = millis();
    uint32_t since = (uint32_t)server.arg("since").toInt();
    if (evSeq - since > (uint32_t)EV_MAX) since = evSeq - EV_MAX;
    String out = String(evSeq) + "\n";
    for (uint32_t i = since; i != evSeq; i++) {
      out += evBuf[i % EV_MAX];
      out += '\n';
    }
    sendText(200, out);
  });
  server.on("/ota", HTTP_POST, []() {
    if (!guard()) { otaShow = false; return; }
    otaShow = false;
    if (httpOtaOk) {
      sendText(200, "OTADONE");
      otaDoneMs = millis();
    } else {
      sendText(500, String("ERR ota ") + Update.errorString());
    }
    dirty = true;
  }, []() {
    HTTPUpload& up = server.upload();
    if (!authed()) return;
    if (up.status == UPLOAD_FILE_START) {
      httpOtaOk = false;
      httpOtaDone = 0;
      httpOtaTotal = server.clientContentLength();
      otaShow = true;
      otaPct = 0;
      wake();
      Update.begin(UPDATE_SIZE_UNKNOWN);
    } else if (up.status == UPLOAD_FILE_WRITE) {
      Update.write(up.buf, up.currentSize);
      httpOtaDone += up.currentSize;
      static uint32_t lastDraw = 0;
      if (httpOtaTotal && millis() - lastDraw > 250) {
        lastDraw = millis();
        otaPct = (int)((uint64_t)httpOtaDone * 100 / httpOtaTotal);
        render();
      }
    } else if (up.status == UPLOAD_FILE_END) {
      httpOtaOk = Update.end(true);
    } else if (up.status == UPLOAD_FILE_ABORTED) {
      Update.abort();
      otaShow = false;
    }
  });
}

static void wifiPoll() {
  if (!wifiOn || !wifiSsid.length()) return;
  uint32_t now = millis();
  if (wifiTrying) {
    if (WiFi.status() == WL_CONNECTED) {
      wifiTrying = false;
      wifiUp = true;
      wifiIp = WiFi.localIP().toString();
      wifiMsg = "Up";
      if (!serverStarted) {
        MDNS.begin("revenant");
        server.begin();
        serverStarted = true;
      }
      emit(String("WIFI up ") + wifiIp);
      dirty = true;
    } else if (now - wifiStartMs > 20000) {
      wifiTrying = false;
      WiFi.disconnect();
      wifiMsg = "Failed";
      wifiRetryMs = now;
      emit("WIFI fail");
      dirty = true;
    }
  } else if (wifiUp && WiFi.status() != WL_CONNECTED) {
    wifiUp = false;
    wifiMsg = "Lost";
    wifiRetryMs = now;
    dirty = true;
  } else if (!wifiUp && now - wifiRetryMs > 15000) {
    wifiBegin();
  }
  if (wifiUp) server.handleClient();
}

// ---------- saved preferences, screen sleep and wake ----------

static void prefsSave() {
  File f = LittleFS.open("/prefs.txt", "w");
  if (!f) return;
  f.printf("%d %d %d %d %d %d %d %d %d %d %d %d %d\n", brIdx, gapIdx, sleepIdx, wifiOn ? 1 : 0, tolAbs, tolPct, prefixOn ? 1 : 0, minMarkUs, shapeTol, sendWaitMs, sendRepeats, txDuty, bleKbMode ? 1 : 0);
  f.close();
}

static void prefsLoad() {
  File f = LittleFS.open("/prefs.txt", "r");
  if (!f) return;
  String l = f.readStringUntil('\n');
  f.close();
  int b, g, sl, w, ta = tolAbs, tp = tolPct, px = prefixOn ? 1 : 0, mm = minMarkUs, sh = shapeTol, sw = sendWaitMs, sr = sendRepeats, td = txDuty, bk = 0;
  int got = sscanf(l.c_str(), "%d %d %d %d %d %d %d %d %d %d %d %d %d", &b, &g, &sl, &w, &ta, &tp, &px, &mm, &sh, &sw, &sr, &td, &bk);
  if (got >= 4) {
    if (got >= 8) {
      if (got >= 9 && sh >= 0 && sh <= 8) shapeTol = sh;
      if (got >= 10 && sw >= 0 && sw <= 1000) sendWaitMs = sw;
      if (got >= 12 && sr >= 1 && sr <= 6) sendRepeats = sr;
      if (got >= 12 && td >= 20 && td <= 66) txDuty = td;
      if (got >= 13) { bleKbMode = bk != 0; bleModeSaved = true; }
      if (ta >= 50 && ta <= 1000) tolAbs = ta;
      if (tp >= 0 && tp <= 50) tolPct = tp;
      prefixOn = px != 0;
      if (mm >= 100 && mm <= 1000) minMarkUs = mm;
    }
    if (b >= 0 && b < 5) brIdx = b;
    if (g >= 0 && g < 3) gapIdx = g;
    if (sl >= 0 && sl < 6) sleepIdx = sl;
    wifiOn = w != 0;
  }
}

static void wake() {
  lastActMs = millis();
  if (!asleep) return;
  asleep = false;
  M5.Display.setBrightness(BRIGHT[brIdx]);
  dirty = true;
}

static void goSleep() {
  asleep = true;
  menuOpen = false;
  M5.Display.setBrightness(0);
}

// lifting the stick or any press wakes it, and the timer restarts on use
static void sleepPoll() {
  static uint32_t lastImu = 0;
  static float base[3];
  static bool have = false;
  uint32_t now = millis();
  if (now - lastImu >= 60 && M5.Imu.isEnabled()) {
    lastImu = now;
    float v[3], gy[3] = {0, 0, 0};
    if (M5.Imu.getAccel(&v[0], &v[1], &v[2])) {
      M5.Imu.getGyro(&gy[0], &gy[1], &gy[2]);
      if (!have) { memcpy(base, v, sizeof v); have = true; }
      float d = fabsf(v[0] - base[0]) + fabsf(v[1] - base[1]) + fabsf(v[2] - base[2]);
      float spin = fabsf(gy[0]) + fabsf(gy[1]) + fabsf(gy[2]);  // degrees per second
      for (int i = 0; i < 3; i++) base[i] = base[i] * 0.9f + v[i] * 0.1f;
      if (d > imuMaxD) imuMaxD = d;
      if (spin > imuMaxSpin) imuMaxSpin = spin;
      imuSeen = true;
      if (d > 0.12f || spin > 40.0f) {
        if (asleep) wake();
        else lastActMs = now;
      }
    }
  }
  uint32_t lim = (uint32_t)SLEEP_S[sleepIdx] * 1000UL;
  if (!asleep && lim && now - lastActMs > lim && !otaActive && !otaShow && !otaDoneMs) goSleep();
}

// ---------- buttons ----------

static void powerDown() {
  M5.Display.fillScreen(TFT_BLACK);
  M5.Display.setTextSize(2);
  M5.Display.setTextColor(C_TEXT, TFT_BLACK);
  M5.Display.setCursor(10, 110);
  M5.Display.print("Power Off");
  delay(500);
  M5.Power.powerOff();
}

static void menuChange() {
  if (menuSel == 0) {
    brIdx = (brIdx + 1) % 5;
    M5.Display.setBrightness(BRIGHT[brIdx]);
  } else if (menuSel == 1) {
    sleepIdx = (sleepIdx + 1) % 6;
  } else if (menuSel == 2) {
    wifiPower(!wifiOn);
  } else if (menuSel == 3) {
    bleKbMode = !bleKbMode;  // Bluetooth is a keyboard for TVs or the setup link, the stick restarts to switch
    bleModeSaved = true;
    restartAt = millis() + 500;
  } else if (menuSel == 4) {
    gapIdx = (gapIdx + 1) % 3;
  } else if (menuSel == 5) {
    ble_store_clear();  // the TV and phones must pair again
    forgetShownMs = millis();
  } else {
    menuOpen = false;
  }
  prefsSave();
  dirty = true;
}

static void handleButtons() {
  if (M5.BtnPWR.wasReleased() && M5.Power.M5pm1.wasPekDoubleClicked()) powerDown();
  if (asleep) {  // the first press only wakes the screen
    if (M5.BtnA.wasPressed() || M5.BtnB.wasPressed()) wake();
    return;
  }
  if (M5.BtnA.wasPressed() || M5.BtnB.wasPressed()) lastActMs = millis();
  if (M5.BtnB.wasPressed()) {
    if (!menuOpen) {
      menuOpen = true;
      menuSel = 0;
    } else {
      menuSel = (menuSel + 1) % MENU_ITEMS;
    }
    dirty = true;
  }
  if (M5.BtnA.wasPressed()) {
    if (menuOpen) {
      menuChange();
    } else if (bleConnected && !windowOpen()) {
      openUntil = millis() + 120000;  // physical press: this is you, not someone nearby
      dirty = true;
    } else if (cap.valid) {
      snprintf(lastTxName, sizeof lastTxName, "Echo");
      irSend(38000, cap.t, cap.n, 1, 0);
    }
  }
}

// ---------- main ----------

void setup() {
  auto cfg = M5.config();
  cfg.internal_spk = false;
  cfg.internal_mic = false;
  cfg.internal_imu = true;  // the motion sensor wakes the screen when the stick is picked up
  Serial.begin(115200);
  Serial.setTxTimeoutMs(0);
  xTaskCreate(beatTask, "beat", 3072, nullptr, 1, nullptr);
  M5.begin(cfg);
  stage = 1;
  M5.Speaker.end();
  size_t heapBefore = heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL);
  LittleFS.begin(true);
  prefsLoad();  // the saved Bluetooth mode decides what the radio becomes
  wifiLoad();
  if (!bleModeSaved && wifiSsid.length()) bleKbMode = true;  // already set up on Wi-Fi, so Bluetooth is for the TV
  bleStart(bleKbMode);  // the radio needs a big block of internal RAM, so it starts before anything large is allocated
  size_t heapAfter = heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL);
  stage = 2;
  rxUp = rxInit();
  stage = 3;
  txUp = txInit();
  M5.Ex_I2C.release();  // the Grove port belongs to the IR module now
  rx2Up = rx2Init();
  stage = 4;
  M5.Power.setExtOutput(true, m5::ext_none);  // IR receiver and transmitter are on the switched 5V
  stage = 5;

  M5.Display.setRotation(0);
  M5.Display.setBrightness(BRIGHT[brIdx]);
  M5.Display.fillScreen(TFT_BLACK);
  W = M5.Display.width();
  H = M5.Display.height();
  cv.setColorDepth(16);
  cv.createSprite(W, H);

  stage = 6;
  delay(250);
  idleLevel = gpio_get_level((gpio_num_t)IR_RX_PIN);
  idleLevel2 = gpio_get_level((gpio_num_t)IR_RX2_PIN);
  if (rx2Up && idleLevel2 != idleLevel) rx2Up = false;  // a receiver that does not idle like the built-in one is not there or not ready
  if (rxUp) rxArm();
  if (rx2Up) rx2Arm();

  stage = 7;
  M5.Display.setBrightness(BRIGHT[brIdx]);
  lastActMs = millis();
  rulesLoad();
  keyLoad();
  blLoad();
  wifiRoutes();
  if (wifiSsid.length() && wifiOn) wifiBegin();
  stage = 8;
  batt = M5.Power.getBatteryLevel();
  Serial.printf("Revenant %s in=%d out=%d blockBefore=%u blockAfter=%u\n", FW_VERSION, rxUp, txUp, (unsigned)heapBefore, (unsigned)heapAfter);
  render();
  stage = 9;
}

void loop() {
  loopCount++;
  M5.update();
  sleepPoll();
  static bool bleWas = false;
  if (bleConnected && !bleWas) wake();  // someone is connecting, show the screen
  bleWas = bleConnected;
  if (!otaActive) {
    handleButtons();
    rxPoll();
    mapRun();
  }
  pollCommands();
  wifiPoll();
  blPoll();
  otaPump();
  if (otaDoneMs && millis() - otaDoneMs > 1500) ESP.restart();
  if (restartAt && (int32_t)(millis() - restartAt) > 0) ESP.restart();
  if (windowOpen() != wasOpen) {
    wasOpen = windowOpen();
    emit(wasOpen ? "UNLOCKED" : "LOCKED");
    dirty = true;
  }
  if (kbConnected != kbReported) {
    kbReported = kbConnected;
    emit(kbConnected ? "KB 1" : "KB 0");
    dirty = true;
  }
  bleFlush();
  if (millis() - lastBattMs > 10000) {
    lastBattMs = millis();
    batt = M5.Power.getBatteryLevel();
    dirty = true;
  }
  if (dirty && !asleep && millis() - lastDrawMs >= 50) {
    lastDrawMs = millis();
    dirty = false;
    render();
    drawCount++;
  }
  static uint32_t lastBeat = 0;
  if (millis() - lastBeat >= 2000) {
    lastBeat = millis();
    Serial.printf("t=%lus draws=%lu reset=%d ble=%d heap=%u imu=%d seen=%d maxd=%.2f maxspin=%.0f asleep=%d\n", (unsigned long)(millis() / 1000), (unsigned long)drawCount, (int)esp_reset_reason(), (int)bleConnected, (unsigned)ESP.getFreeHeap(), (int)M5.Imu.isEnabled(), (int)imuSeen, imuMaxD, imuMaxSpin, (int)asleep);
    imuMaxD = imuMaxSpin = 0;
  }
  delay(2);
}
