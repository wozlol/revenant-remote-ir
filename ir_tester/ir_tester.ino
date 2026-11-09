// Super Remote, IR raw tester 0.1, M5StickS3
// Front button: clear. Right button: menu. Power button: double click for full off.
// Silent by design. Speaker, mic and IMU are never started.

#include <M5Unified.h>
#include "driver/rmt_rx.h"

#define FW_VERSION "0.1"
#define C565(r, g, b) ((((r) & 0xF8) << 8) | (((g) & 0xFC) << 3) | ((b) >> 3))

static const int IR_RX_PIN = 42;
static const int IR_TX_PIN = 46;
static const size_t RX_SYMS = 256;
static const uint16_t MAX_T = 512;
static const uint16_t MIN_MARK_US = 300;  // shorter marks are backlight noise, real remotes use 400 us or more
static const int WAVE_ROWS = 8;
static const int WAVE_ROW_H = 24;
static const int WAVE_Y0 = 18;

static const uint16_t C_TEXT = C565(220, 220, 220);
static const uint16_t C_DIM = C565(110, 110, 110);
static const uint16_t C_LINE = C565(40, 40, 40);
static const uint16_t C_MARK = C565(0, 210, 190);
static const uint16_t C_SPACE = C565(120, 120, 120);
static const uint16_t C_HI = C565(255, 190, 0);
static const uint16_t C_NEW = C565(0, 230, 90);
static const uint16_t C_SAME = C565(255, 150, 0);

struct Capture {
  uint16_t t[MAX_T];  // microseconds, mark first, then alternating space and mark
  uint16_t n;
  uint32_t total;
  uint32_t seq;
  uint8_t burst;
  uint32_t repeatMs;
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
static const char* const VIEW_NAME[] = {"Summary", "Waveform", "Timings"};
static const int MENU_ITEMS = 4;

static M5Canvas cv(&M5.Display);
static Capture cap;
static rmt_symbol_word_t rxBuf[RX_SYMS];
static volatile size_t rxNum = 0;
static volatile bool rxDone = false;
static rmt_channel_handle_t rxChan = nullptr;
static int pinLast = 1;
static uint32_t edgeCount = 0;
static uint16_t tmpT[MAX_T];

static int W = 135;
static int H = 240;
static int batt = -1;
static int idleLevel = 1;
static bool rmtUp = false;
static uint8_t viewIdx = 0;
static uint8_t brIdx = 1;
static uint8_t gapIdx = 1;
static bool menuOpen = false;
static uint8_t menuSel = 0;
static uint32_t noiseCount = 0;
static uint32_t lastFrameMs = 0;
static uint32_t lastBattMs = 0;
static uint32_t lastDrawMs = 0;
static bool dirty = true;
static bool isNew = true;
static uint32_t newCount = 0;

// ---------- IR capture ----------

static bool IRAM_ATTR onRecvDone(rmt_channel_handle_t ch, const rmt_rx_done_event_data_t* e, void* user) {
  rxNum = e->num_symbols;
  rxDone = true;
  return false;
}

static bool irInit() {
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

static void irArm() {
  rmt_receive_config_t rc = {};
  rc.signal_range_min_ns = 1000;
  rc.signal_range_max_ns = (uint32_t)GAP_US[gapIdx] * 1000;
  rxDone = false;
  rmt_receive(rxChan, rxBuf, sizeof(rxBuf), &rc);
}

static bool nearT(uint16_t v, uint16_t target) {
  int d = (int)v - (int)target;
  if (d < 0) d = -d;
  return d <= (int)target * 25 / 100;
}

static bool allUnits(const Capture& c, uint16_t unit) {
  for (uint16_t i = 0; i < c.n; i++) {
    if (!nearT(c.t[i], unit) && !nearT(c.t[i], unit * 2)) return false;
  }
  return true;
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
  } else if (c.n >= 10 && allUnits(c, 889)) {
    g.name = "RC-5";
  }
  return g;
}

static uint16_t frT[MAX_T];
static uint16_t refT[MAX_T];  // first frame of the previous press
static uint16_t refN = 0;

static bool sameCode(const uint16_t* a, const uint16_t* b, uint16_t n) {
  for (uint16_t i = 0; i < n; i++) {
    int d = (int)a[i] - (int)b[i];
    if (d < 0) d = -d;
    int lim = (int)b[i] / 5;
    if (lim < 120) lim = 120;
    if (d > lim) return false;
  }
  return true;
}

static void commitFrame(const uint16_t* t, uint16_t n) {
  if (n < 3 || (n < 9 && t[0] < 1500)) {
    noiseCount++;
    return;
  }
  uint32_t now = millis();
  if (cap.valid && (now - lastFrameMs) < 250) {
    cap.burst++;
    cap.repeatMs = now - lastFrameMs;
  } else {
    cap.burst = 1;
    cap.repeatMs = 0;
    isNew = !(refN == n && sameCode(t, refT, n));
    if (isNew) newCount++;
    memcpy(refT, t, n * sizeof(uint16_t));
    refN = n;
  }
  lastFrameMs = now;
  memcpy(cap.t, t, n * sizeof(uint16_t));
  cap.n = n;
  cap.total = 0;
  for (uint16_t i = 0; i < n; i++) cap.total += cap.t[i];
  cap.seq++;
  cap.valid = true;
  dirty = true;

  Serial.printf("#%lu n=%u total=%luus burst=%u raw:", (unsigned long)cap.seq, cap.n, (unsigned long)cap.total, cap.burst);
  for (uint16_t i = 0; i < cap.n; i++) Serial.printf(" %u", cap.t[i]);
  Serial.println();
}

static void irPoll() {
  int lv = gpio_get_level((gpio_num_t)IR_RX_PIN);
  if (lv != pinLast) {
    pinLast = lv;
    edgeCount++;
    if (!cap.valid) dirty = true;
  }
  if (!rmtUp || !rxDone) return;
  size_t rxLen = rxNum;

  bool markLevel = !idleLevel;
  uint16_t n = 0;
  for (size_t i = 0; i < rxLen && n < MAX_T; i++) {
    rmt_symbol_word_t s = rxBuf[i];
    if (s.duration0 == 0) break;
    if (!(n == 0 && (bool)s.level0 != markLevel)) tmpT[n++] = s.duration0;
    if (s.duration1 == 0 || n >= MAX_T) break;
    if (!(n == 0 && (bool)s.level1 != markLevel)) tmpT[n++] = s.duration1;
  }

  irArm();

  // Drop marks shorter than MIN_MARK_US by folding them into the surrounding space.
  // A folded space as long as the frame end gap splits one capture into separate frames.
  uint32_t gapUs = GAP_US[gapIdx];
  uint32_t pending = 0;
  uint16_t fn = 0;
  for (uint16_t i = 0; i < n; i++) {
    bool mark = (i % 2 == 0);
    if (!mark) {
      pending += tmpT[i];
      continue;
    }
    if (tmpT[i] < MIN_MARK_US) {
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

static void clearCapture() {
  cap.valid = false;
  cap.n = 0;
  cap.seq = 0;
  cap.burst = 0;
  cap.repeatMs = 0;
  noiseCount = 0;
  newCount = 0;
  refN = 0;
  isNew = true;
  dirty = true;
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

static void drawFooter(const char* s) {
  txt(2, H - 12, 1, C_DIM, s);
}

static void drawRowC(int y, const char* label, const char* val, uint16_t col) {
  txt(2, y, 1, C_DIM, label);
  txt(60, y, 1, col, val);
}

static void drawRow(int y, const char* label, const char* val) {
  drawRowC(y, label, val, C_TEXT);
}

static void drawTimingLines(int y0, int firstIdx, int lines) {
  for (int l = 0; l < lines; l++) {
    for (int k = 0; k < 4; k++) {
      int i = firstIdx + l * 4 + k;
      if (i >= cap.n) return;
      char b[8];
      snprintf(b, sizeof b, "%5u", cap.t[i]);
      txt(2 + k * 30, y0 + l * 10, 1, (i % 2 == 0) ? C_MARK : C_SPACE, b);
    }
  }
}

static void drawWaiting() {
  txt(2, 28, 2, C_TEXT, "Waiting");
  txt(2, 54, 1, C_DIM, "Point a remote at the");
  txt(2, 64, 1, C_DIM, "stick and press a");
  txt(2, 74, 1, C_DIM, "button.");
  char b[32];
  snprintf(b, sizeof b, "%s", idleLevel ? "HIGH" : "LOW");
  drawRow(100, "Idle", b);
  drawRow(136, "Pin now", pinLast ? "HIGH" : "LOW");
  snprintf(b, sizeof b, "%lu", (unsigned long)edgeCount);
  drawRow(148, "Edges", b);
  drawRow(112, "Frame end", GAP_NAME[gapIdx]);
  if (!rmtUp) txt(2, 164, 1, C_HI, "RMT failed to start");
  snprintf(b, sizeof b, "%lu", (unsigned long)noiseCount);
  drawRow(124, "Noise", b);
}

static void drawSummary() {
  Guess g = guessProtocol(cap);
  uint16_t nc = isNew ? C_NEW : C_SAME;
  txt(2, 18, 2, nc, g.name);
  char b[40];
  snprintf(b, sizeof b, "#%lu %s %lu", (unsigned long)cap.seq, isNew ? "NEW" : "SAME", (unsigned long)newCount);
  drawRowC(42, "Frame", b, nc);
  snprintf(b, sizeof b, "%u", cap.burst);
  drawRow(53, "Burst", b);
  snprintf(b, sizeof b, "%u", cap.n);
  drawRow(64, "Timings", b);
  if (g.bits >= 0) {
    snprintf(b, sizeof b, "%d", g.bits);
    drawRow(75, "Bits", b);
  }
  snprintf(b, sizeof b, "%lu.%lu ms", (unsigned long)(cap.total / 1000), (unsigned long)((cap.total % 1000) / 100));
  drawRow(86, "Length", b);
  snprintf(b, sizeof b, "%u / %u", cap.t[0], cap.t[1]);
  drawRow(97, "Lead us", b);
  if (cap.burst > 1) {
    snprintf(b, sizeof b, "%lu ms", (unsigned long)cap.repeatMs);
    drawRow(108, "Repeat", b);
  }
  snprintf(b, sizeof b, "%lu", (unsigned long)noiseCount);
  drawRow(119, "Noise", b);
  txt(2, 138, 1, C_DIM, "Raw us");
  drawTimingLines(150, 0, 7);
}

static void drawWave() {
  int rowW = W - 4;
  uint32_t upp = cap.total / (uint32_t)(WAVE_ROWS * rowW) + 1;
  if (upp < 10) upp = 10;
  uint32_t cum = 0;
  for (uint16_t i = 0; i < cap.n; i++) {
    bool mark = (i % 2 == 0);
    uint32_t a = cum / upp;
    uint32_t b = (cum + cap.t[i]) / upp;
    if (mark && b <= a) b = a + 1;
    cum += cap.t[i];
    for (uint32_t p = a; p < b; p++) {
      int row = p / rowW;
      if (row >= WAVE_ROWS) break;
      int x = 2 + (p % rowW);
      int base = WAVE_Y0 + row * WAVE_ROW_H + 20;
      if (mark) cv.drawFastVLine(x, base - 16, 17, C_MARK);
      else cv.drawPixel(x, base, C_SPACE);
    }
  }
  char s[24];
  snprintf(s, sizeof s, "1 px = %lu us", (unsigned long)upp);
  txt(2, WAVE_Y0 + WAVE_ROWS * WAVE_ROW_H + 2, 1, C_DIM, s);
}

static void drawTimes() {
  drawTimingLines(WAVE_Y0, 0, 21);
}

static void drawMenu() {
  drawHeader("MENU");
  const char* labels[MENU_ITEMS] = {"View", "Brightness", "Frame end", ""};
  const char* values[MENU_ITEMS] = {VIEW_NAME[viewIdx], BRIGHT_NAME[brIdx], GAP_NAME[gapIdx], "Close"};
  for (int i = 0; i < MENU_ITEMS; i++) {
    int y = 24 + i * 44;
    uint16_t col = (i == menuSel) ? C_HI : C_TEXT;
    if (i == menuSel) cv.fillRect(0, y, 2, 30, C_HI);
    txt(6, y, 1, C_DIM, labels[i]);
    txt(6, y + 12, 2, col, values[i]);
  }
  drawFooter("Right Next Front Pick");
}

static void render() {
  cv.fillSprite(TFT_BLACK);
  cv.setFont(&fonts::Font0);
  if (menuOpen) {
    drawMenu();
  } else {
    drawHeader("IR RAW v" FW_VERSION);
    if (cap.valid) cv.fillRect(0, 13, W, 4, isNew ? C_NEW : C_SAME);
    if (!cap.valid) drawWaiting();
    else if (viewIdx == 0) drawSummary();
    else if (viewIdx == 1) drawWave();
    else drawTimes();
    drawFooter("Front Clear Right Menu");
  }
  cv.pushSprite(0, 0);
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
    viewIdx = (viewIdx + 1) % 3;
  } else if (menuSel == 1) {
    brIdx = (brIdx + 1) % 5;
    M5.Display.setBrightness(BRIGHT[brIdx]);
  } else if (menuSel == 2) {
    gapIdx = (gapIdx + 1) % 3;
  } else {
    menuOpen = false;
  }
  dirty = true;
}

static void handleButtons() {
  if (M5.BtnPWR.wasReleased() && M5.Power.M5pm1.wasPekDoubleClicked()) powerDown();
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
    if (menuOpen) menuChange();
    else clearCapture();
  }
}

// ---------- main ----------

void setup() {
  auto cfg = M5.config();
  cfg.internal_spk = false;
  cfg.internal_mic = false;
  cfg.internal_imu = false;
  M5.begin(cfg);
  M5.Speaker.end();
  rmtUp = irInit();
  M5.Power.setExtOutput(true, m5::ext_none);  // IR receiver and transmitter are on the switched 5V

  Serial.begin(115200);
  Serial.setTxTimeoutMs(0);

  pinMode(IR_TX_PIN, OUTPUT);
  digitalWrite(IR_TX_PIN, LOW);  // keep the transmit LED off

  M5.Display.setRotation(0);
  M5.Display.setBrightness(BRIGHT[brIdx]);
  M5.Display.fillScreen(TFT_BLACK);
  W = M5.Display.width();
  H = M5.Display.height();
  cv.setColorDepth(16);
  cv.createSprite(W, H);

  delay(250);
  idleLevel = gpio_get_level((gpio_num_t)IR_RX_PIN);
  pinLast = idleLevel;

  clearCapture();
  if (rmtUp) irArm();
  batt = M5.Power.getBatteryLevel();
  Serial.printf("Super Remote IR tester %s idle=%d rmt=%d\n", FW_VERSION, idleLevel, rmtUp);
  render();
}

void loop() {
  M5.update();
  handleButtons();
  irPoll();
  if (millis() - lastBattMs > 10000) {
    lastBattMs = millis();
    batt = M5.Power.getBatteryLevel();
    dirty = true;
  }
  if (dirty && millis() - lastDrawMs >= 50) {
    lastDrawMs = millis();
    dirty = false;
    render();
  }
  delay(2);
}
